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

from succorfish_msgs.msg import SerialFrame, SerialLine, Topics  # noqa: E402
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


def test_tx_topic_waits_until_send_command_finishes(serial_bridge):
    """A topic TX must not land on the wire while SendCommand is awaiting a reply."""
    node, master = serial_bridge

    client_node = rclpy.create_node('tx_session_test_client')
    sub_exec = MultiThreadedExecutor()
    sub_exec.add_node(client_node)
    threading.Thread(target=sub_exec.spin, daemon=True).start()

    try:
        cli = client_node.create_client(SendCommand, Topics.SEND_COMMAND_SERVICE)
        assert cli.wait_for_service(timeout_sec=5.0)

        req = SendCommand.Request()
        req.command = '$P002'
        req.append_terminator = True
        req.expect_regex = r'T\d+'
        req.timeout = 4.0
        future = cli.call_async(req)
        got = bytearray()

        def pump():
            got.extend(_read_available(master))
            return bytes(got)

        assert _wait_until(lambda: b'$P002\r\n' in pump(), timeout=2.0)

        pub = node.create_publisher(String, Topics.TX_TOPIC, 10)
        pub.publish(String(data='$B0459.1,18.8'))
        time.sleep(0.4)
        pump()
        assert b'$B' not in bytes(got), f'$B wrote during in-flight ping: {bytes(got)!r}'

        os.write(master, b'#R002T5678\r\n')
        assert _wait_until(lambda: future.done(), timeout=4.0)
        assert future.result().success
        assert _wait_until(lambda: b'$B0459.1,18.8\r\n' in pump(), timeout=2.0)
    finally:
        sub_exec.shutdown()
        client_node.destroy_node()


def _read_available(fd):
    os.set_blocking(fd, False)
    got = bytearray()
    while True:
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            break
        if not chunk:
            break
        got.extend(chunk)
    return bytes(got)


def test_tx_bytes_written_verbatim_without_terminator(serial_bridge):
    """``succorfish/tx_bytes`` is a raw UART write, including interior LF."""
    node, master = serial_bridge
    pub = node.create_publisher(SerialFrame, Topics.TX_BYTES_TOPIC, 10)
    payload = b"$B05He\nlo"
    msg = SerialFrame()
    msg.data = payload
    pub.publish(msg)

    got = bytearray()

    def saw():
        got.extend(_read_available(master))
        return payload in bytes(got)

    assert _wait_until(saw), f"got {bytes(got)!r}"
    wire = bytes(got)
    idx = wire.find(payload)
    after = wire[idx + len(payload):idx + len(payload) + 2]
    assert after != b"\r\n", f"terminator appended: {wire!r}"


def test_binary_broadcast_on_rx_bytes_not_split_on_rx(serial_bridge):
    """A ``#B`` payload containing LF is one SerialFrame and no SerialLine."""
    node, master = serial_bridge
    frames = []
    lines = []
    node.create_subscription(
        SerialFrame, Topics.RX_BYTES_TOPIC,
        lambda m: frames.append(bytes(m.data)), 10)
    node.create_subscription(
        SerialLine, Topics.RX_TOPIC, lambda m: lines.append(m.line), 10)

    payload = b"He\nlo"
    frame = b"#B00705" + payload
    os.write(master, frame + b"\r\n")

    assert _wait_until(lambda: frame in frames), f"frames={frames!r}"
    time.sleep(0.3)
    assert not any(ln.startswith("#B") for ln in lines), f"lines={lines!r}"


def test_ascii_broadcast_still_on_rx(serial_bridge):
    node, master = serial_bridge
    frames = []
    lines = []
    node.create_subscription(
        SerialFrame, Topics.RX_BYTES_TOPIC,
        lambda m: frames.append(bytes(m.data)), 10)
    node.create_subscription(
        SerialLine, Topics.RX_TOPIC, lambda m: lines.append(m.line), 10)

    raw = b"#B00705Hello"
    os.write(master, raw + b"\r\n")

    assert _wait_until(lambda: "#B00705Hello" in lines), f"lines={lines!r}"
    assert _wait_until(lambda: raw in frames), f"frames={frames!r}"
