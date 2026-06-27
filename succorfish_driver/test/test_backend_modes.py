"""End-to-end tests of the 'test' and 'unity' backends over a real rclpy graph.

Neither backend needs hardware or a serial port:
  * 'test'  -- the driver answers a SendCommand the way a modem would.
  * 'unity' -- the driver bridges modem lines to/from smarc_msgs/StringStamped.
"""

import threading
import time

import pytest

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


def _spin(overrides):
    rclpy.init()
    node = SuccorfishDriver(parameter_overrides=overrides)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    assert _wait_until(lambda: node._connected is True), "driver never connected"
    return node, executor


def _teardown(node, executor):
    node.shutdown()
    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


# --------------------------------------------------------------------------- #
# 'test' backend                                                              #
# --------------------------------------------------------------------------- #

def test_test_backend_answers_succorfish_ping():
    node, executor = _spin([
        Parameter("backend", value="test"),
        Parameter("profile", value="succorfish"),
        Parameter("test.range_m", value=1500.0),
        Parameter("test.sound_velocity", value=1500.0),
        Parameter("reconnect_delay_s", value=0.1),
    ])
    client = rclpy.create_node("ping_client")
    cexec = MultiThreadedExecutor()
    cexec.add_node(client)
    threading.Thread(target=cexec.spin, daemon=True).start()
    try:
        cli = client.create_client(SendCommand, Topics.SEND_COMMAND_SERVICE)
        assert cli.wait_for_service(timeout_sec=5.0)
        req = SendCommand.Request()
        req.command = "$P007"
        req.append_terminator = True
        req.expect_regex = r"T\d+"
        req.timeout = 3.0
        future = cli.call_async(req)
        assert _wait_until(lambda: future.done(), timeout=6.0)
        resp = future.result()
        assert resp.success
        assert resp.matched_line == "#R007T32000"  # 1500 m -> 32000 ticks
    finally:
        cexec.shutdown()
        client.destroy_node()
        _teardown(node, executor)


def test_test_backend_acks_teensy_config():
    node, executor = _spin([
        Parameter("backend", value="test"),
        Parameter("profile", value="teensy"),
        Parameter("reconnect_delay_s", value=0.1),
    ])
    client = rclpy.create_node("cfg_client")
    cexec = MultiThreadedExecutor()
    cexec.add_node(client)
    threading.Thread(target=cexec.spin, daemon=True).start()
    try:
        cli = client.create_client(SendCommand, Topics.SEND_COMMAND_SERVICE)
        assert cli.wait_for_service(timeout_sec=5.0)
        req = SendCommand.Request()
        req.command = "$Y101R"
        req.append_terminator = True
        req.expect_regex = r"#A\d+"
        req.timeout = 3.0
        future = cli.call_async(req)
        assert _wait_until(lambda: future.done(), timeout=6.0)
        assert future.result().matched_line == "#A101"
    finally:
        cexec.shutdown()
        client.destroy_node()
        _teardown(node, executor)


# --------------------------------------------------------------------------- #
# 'unity' backend                                                             #
# --------------------------------------------------------------------------- #

def test_unity_backend_rx_synthesizes_broadcast_and_delta():
    pytest.importorskip("smarc_msgs.msg")
    from smarc_msgs.msg import StringStamped
    node, executor = _spin([
        Parameter("backend", value="unity"),
        Parameter("profile", value="teensy"),
        Parameter("unity.own_modem_id", value="007"),
        Parameter("reconnect_delay_s", value=0.1),
    ])
    client = rclpy.create_node("unity_rx_client")
    cexec = MultiThreadedExecutor()
    cexec.add_node(client)
    threading.Thread(target=cexec.spin, daemon=True).start()
    received = []
    client.create_subscription(
        SerialLine, Topics.RX_TOPIC, lambda m: received.append(m.line), 10)
    pub = client.create_publisher(StringStamped, "acoustic/read", 10)
    try:
        msg = StringStamped()
        msg.data = "042;59.0,18.0"
        msg.time_sent.sec = 10
        msg.time_sent.nanosec = 0
        msg.time_received.sec = 10
        msg.time_received.nanosec = 50_000_000  # 0.05 s time of flight
        assert _wait_until(lambda: pub.get_subscription_count() > 0, timeout=5.0)
        pub.publish(msg)
        assert _wait_until(lambda: "#B0420959.0,18.0" in received), f"got {received!r}"
        assert _wait_until(lambda: "#I50000" in received), f"got {received!r}"
    finally:
        cexec.shutdown()
        client.destroy_node()
        _teardown(node, executor)


def test_unity_backend_acks_teensy_config_locally():
    # The Teensy config handshake ($Y<id><mode> -> #A<id>) is local, not
    # acoustic. A transmitter beacon gates telemetry on this ack, so the unity
    # bridge must answer it locally (over RX) without transmitting it.
    pytest.importorskip("smarc_msgs.msg")
    node, executor = _spin([
        Parameter("backend", value="unity"),
        Parameter("profile", value="teensy"),
        Parameter("reconnect_delay_s", value=0.1),
    ])
    client = rclpy.create_node("unity_cfg_client")
    cexec = MultiThreadedExecutor()
    cexec.add_node(client)
    threading.Thread(target=cexec.spin, daemon=True).start()
    received = []
    client.create_subscription(
        SerialLine, Topics.RX_TOPIC, lambda m: received.append(m.line), 10)
    pub = client.create_publisher(String, Topics.TX_TOPIC, 10)
    try:
        assert _wait_until(lambda: pub.get_subscription_count() > 0, timeout=5.0)
        pub.publish(String(data="$Y101T0001s"))
        assert _wait_until(lambda: "#A101" in received), f"got {received!r}"
    finally:
        cexec.shutdown()
        client.destroy_node()
        _teardown(node, executor)


def test_unity_backend_tx_publishes_stringstamped():
    pytest.importorskip("smarc_msgs.msg")
    from smarc_msgs.msg import StringStamped
    node, executor = _spin([
        Parameter("backend", value="unity"),
        Parameter("profile", value="succorfish"),
        Parameter("unity.own_modem_id", value="003"),
        Parameter("reconnect_delay_s", value=0.1),
    ])
    client = rclpy.create_node("unity_tx_client")
    cexec = MultiThreadedExecutor()
    cexec.add_node(client)
    threading.Thread(target=cexec.spin, daemon=True).start()
    sent = []
    client.create_subscription(
        StringStamped, "acoustic/write", lambda m: sent.append(m.data), 10)
    pub = client.create_publisher(String, Topics.TX_TOPIC, 10)
    try:
        assert _wait_until(lambda: pub.get_subscription_count() > 0, timeout=5.0)
        pub.publish(String(data="$B0959.0,18.0"))
        assert _wait_until(lambda: "003;59.0,18.0" in sent), f"got {sent!r}"
    finally:
        cexec.shutdown()
        client.destroy_node()
        _teardown(node, executor)
