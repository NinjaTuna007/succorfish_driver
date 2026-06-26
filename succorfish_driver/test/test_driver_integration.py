"""End-to-end test of the driver against a virtual serial port (pty).

Exercises the three bridge paths over a real rclpy graph:
  * inbound bytes  -> SerialLine on RX_TOPIC
  * std_msgs/String on TX_TOPIC -> bytes written to the port
  * SendCommand service -> command written, matching reply returned

Skipped automatically if pyserial / rclpy are unavailable.
"""

import os
import threading
import time
import tty

import pytest

serial = pytest.importorskip("serial")
rclpy = pytest.importorskip("rclpy")

from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
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
    master, slave = os.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    slave_name = os.ttyname(slave)

    rclpy.init()
    node = SuccorfishDriver(parameter_overrides=[
        Parameter("serial.port", value=slave_name),
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
        os.close(master)
        try:
            os.close(slave)
        except OSError:
            pass


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
