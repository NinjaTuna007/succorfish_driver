"""End-to-end test of the driver against a virtual serial port (pty).

Exercises the three bridge paths over a real rclpy graph:
  * inbound bytes  -> SerialLine on RX_TOPIC
  * std_msgs/String on TX_TOPIC -> bytes written to the port
  * SendCommand service -> command written, matching reply returned

Skipped automatically if pyserial / rclpy are unavailable.
"""

import os
import shutil
import subprocess
import tempfile
import threading
import time

import pytest

serial = pytest.importorskip("serial")
rclpy = pytest.importorskip("rclpy")

from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from succorfish_msgs.msg import SerialLine, Topics  # noqa: E402
from succorfish_msgs.srv import SendCommand  # noqa: E402
from succorfish_driver.succorfish_driver_node import SuccorfishDriver  # noqa: E402


def _wait_until(predicate, timeout=5.0, period=0.02):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(period)
    return predicate()


@pytest.fixture
def serial_bridge():
    # Back the virtual port with a socat-linked pty pair rather than a bare
    # ``os.openpty()``. A bare pty read through pyserial 3.5 intermittently trips
    # "device reports readiness to read but returned no data", which the driver
    # treats as a disconnect and reconnects in a loop -- so the link is often
    # down (``_connected`` False) at shutdown and the registered shutdown command
    # is never written. socat keeps both ends open and behaves like a real tty.
    socat = shutil.which("socat")
    if socat is None:
        pytest.skip("socat not available")
    tmp = tempfile.mkdtemp(prefix="drvtest_")
    driver_link = os.path.join(tmp, "driver")   # the driver opens this end
    host_link = os.path.join(tmp, "host")        # the test drives this end
    proc = subprocess.Popen(
        [socat,
         f"pty,raw,echo=0,link={driver_link}",
         f"pty,raw,echo=0,link={host_link}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.time() + 5.0
    while time.time() < deadline and not (
            os.path.exists(driver_link) and os.path.exists(host_link)):
        time.sleep(0.02)
    assert os.path.exists(driver_link) and os.path.exists(host_link), \
        "socat did not create the pty links"
    master = os.open(host_link, os.O_RDWR | os.O_NOCTTY)

    rclpy.init()
    node = SuccorfishDriver(parameter_overrides=[
        Parameter("serial.port", value=driver_link),
        Parameter("serial.port_fallback", value=""),
        Parameter("serial.baudrate", value=9600),
        Parameter("reconnect_delay_s", value=0.1),
    ])
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    assert _wait_until(lambda: node._connected is True), "driver never connected to pty"

    try:
        yield node, master
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            os.close(master)
        except OSError:
            pass
        try:
            proc.terminate()
            proc.wait(timeout=2.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
        shutil.rmtree(tmp, ignore_errors=True)


def test_inbound_line_is_published_on_rx(serial_bridge):
    node, master = serial_bridge
    received = []
    node.create_subscription(
        SerialLine, Topics.RX_TOPIC, lambda m: received.append(m.line), 10)

    os.write(master, b"#R001T1234\r\n")

    assert _wait_until(lambda: "#R001T1234" in received), f"got {received!r}"


def test_tx_topic_writes_to_port(serial_bridge):
    node, master = serial_bridge
    pub = node.create_publisher(String, Topics.TX_TOPIC, 10)

    pub.publish(String(data="$P001"))

    got = bytearray()

    def saw_command():
        try:
            os.set_blocking(master, False)
            chunk = os.read(master, 1024)
        except BlockingIOError:
            chunk = b""
        if chunk:
            got.extend(chunk)
        return b"$P001\r\n" in bytes(got)

    assert _wait_until(saw_command), f"got {bytes(got)!r}"


def test_shutdown_command_written_to_port_on_exit(serial_bridge):
    """A registered shutdown command is written to the wire on graceful exit.

    This is the wire-mode guarantee: even if the driver is the one going away,
    it replays the client-registered command (e.g. ``$Y<id>W``) before closing.
    """
    node, master = serial_bridge
    latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(String, Topics.SHUTDOWN_COMMAND_TOPIC, latched)
    pub.publish(String(data="$Y101W"))

    assert _wait_until(lambda: node._shutdown_command == "$Y101W"), \
        "driver never registered the shutdown command"

    # Drain anything already buffered on the master before we trigger shutdown.
    os.set_blocking(master, False)
    try:
        os.read(master, 4096)
    except BlockingIOError:
        pass

    node.shutdown()

    got = bytearray()
    deadline = time.time() + 2.0
    while time.time() < deadline:
        try:
            chunk = os.read(master, 4096)
        except BlockingIOError:
            chunk = b""
        if chunk:
            got.extend(chunk)
        if b"$Y101W\r\n" in bytes(got):
            break
        time.sleep(0.02)

    assert b"$Y101W\r\n" in bytes(got), f"got {bytes(got)!r}"


def test_send_command_returns_matching_reply(serial_bridge):
    node, master = serial_bridge

    client_node = rclpy.create_node("send_command_test_client")
    sub_exec = MultiThreadedExecutor()
    sub_exec.add_node(client_node)
    client_thread = threading.Thread(target=sub_exec.spin, daemon=True)
    client_thread.start()

    try:
        cli = client_node.create_client(SendCommand, Topics.SEND_COMMAND_SERVICE)
        assert cli.wait_for_service(timeout_sec=5.0), "SendCommand service not available"

        req = SendCommand.Request()
        req.command = "$P002"
        req.append_terminator = True
        req.expect_regex = r"T\d+"
        req.timeout = 3.0

        def feed_reply():
            time.sleep(0.3)
            os.write(master, b"#R002T5678\r\n")

        threading.Thread(target=feed_reply, daemon=True).start()

        future = cli.call_async(req)
        assert _wait_until(lambda: future.done(), timeout=6.0), "service call timed out"

        resp = future.result()
        assert resp.success
        assert "T5678" in resp.matched_line
        assert any("T5678" in line for line in resp.lines)
    finally:
        sub_exec.shutdown()
        client_node.destroy_node()
