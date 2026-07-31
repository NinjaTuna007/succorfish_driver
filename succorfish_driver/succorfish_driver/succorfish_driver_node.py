"""Transparent serial bridge for the Succorfish modem / Teensy front-end.

This node exclusively owns one serial port and exposes it to the rest of the
ROS graph as a protocol-agnostic line pipe:

* publishes every inbound line on ``RX_TOPIC`` (``succorfish_msgs/SerialLine``),
* writes any raw command received on ``TX_TOPIC`` (``std_msgs/String``),
* reports link state on ``STATUS_TOPIC`` (``std_msgs/Bool``, latched),
* offers a generic ``SendCommand`` service for synchronous request/response
  (write a command, optionally wait for a reply line matching a regex).

Protocol framing (``$P``, ``$B``, ``#R...T...``, ``#I``, ...) is intentionally
NOT understood here; the existing parser/builder helpers in ``serial_ping_pkg``
stay client-side. A single background thread does all reading so the serial
port has exactly one owner; writes are serialized with a lock.
"""

import signal
import threading
import time
from collections import deque

import rclpy
import serial
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from succorfish_msgs.msg import SerialLine, Topics
from succorfish_msgs.srv import SendCommand

from succorfish_driver.line_assembler import LineAssembler, first_match


class SuccorfishDriver(Node):
    """Owns the serial link and bridges it to ROS topics + a service."""

    def __init__(self, **kwargs):
        super().__init__("succorfish_driver_node", **kwargs)

        self.declare_parameter("serial.port", "/dev/ttyUSB0")
        self.declare_parameter("serial.port_fallback", "/dev/ttyUSB1")
        self.declare_parameter("serial.port_override", "")
        self.declare_parameter("serial.baudrate", 9600)
        self.declare_parameter("serial.timeout", 1.0)
        self.declare_parameter("command_terminator", "\r\n")
        self.declare_parameter("encoding", "utf-8")
        self.declare_parameter("reconnect_delay_s", 2.0)
        self.declare_parameter("recent_lines_buffer", 200)
        self.declare_parameter("profile", "succorfish")

        # Where the bytes come from. 'serial' (default) is the real port;
        # 'test'/'dummy' is an in-memory pretend modem; 'unity' bridges the
        # smarcUnity acoustic transceiver. See backends.py.
        self.declare_parameter("backend", "serial")
        # 'test' backend knobs.
        self.declare_parameter("test.range_m", 100.0)
        self.declare_parameter("test.sound_velocity", 1500.0)
        self.declare_parameter("test.fault.drop_prob", 0.0)
        self.declare_parameter("test.fault.garble_prob", 0.0)
        # 'unity' backend knobs.
        self.declare_parameter("unity.write_topic", "acoustic/write")
        self.declare_parameter("unity.read_topic", "acoustic/read")
        # Dynamic typing: a numeric-looking id like 007 is parsed as an int by
        # ros2 run/launch, so accept any type and normalize it below.
        self.declare_parameter(
            "unity.own_modem_id", "001",
            ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("unity.offset_us", 0.0)
        self.declare_parameter("unity.id_width", 3)

        gp = self.get_parameter
        self.port = gp("serial.port").get_parameter_value().string_value
        self.port_fallback = gp("serial.port_fallback").get_parameter_value().string_value
        self.port_override = gp("serial.port_override").get_parameter_value().string_value
        self.baudrate = gp("serial.baudrate").get_parameter_value().integer_value
        self.serial_timeout = gp("serial.timeout").get_parameter_value().double_value
        self.command_terminator = gp("command_terminator").get_parameter_value().string_value
        self.encoding = gp("encoding").get_parameter_value().string_value
        self.reconnect_delay_s = gp("reconnect_delay_s").get_parameter_value().double_value
        buf_len = gp("recent_lines_buffer").get_parameter_value().integer_value
        self.profile = gp("profile").get_parameter_value().string_value
        self.backend = gp("backend").get_parameter_value().string_value.lower()

        self._assembler = LineAssembler(encoding=self.encoding)
        self._ser = None
        self._connected = None  # None = unknown, forces first status publish
        self._running = True
        self._stop_event = threading.Event()  # wakes the reconnect sleep on shutdown
        # Opaque command a client asks us to write right before we close the port
        # on a graceful exit (e.g. an OWTT node registering "$Y<id>W" wire mode).
        # The driver stays protocol-agnostic: it just replays whatever string was
        # last registered, while it is still the holder of the open port.
        self._shutdown_command = ""
        self._write_lock = threading.Lock()
        self._cond = threading.Condition()
        self._seq = 0
        self._recent = deque(maxlen=max(buf_len, 1))

        cb = ReentrantCallbackGroup()

        self.rx_pub = self.create_publisher(SerialLine, Topics.RX_TOPIC, 10)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(Bool, Topics.STATUS_TOPIC, latched)

        self.tx_sub = self.create_subscription(
            String, Topics.TX_TOPIC, self._on_tx, 10, callback_group=cb)
        self.send_srv = self.create_service(
            SendCommand, Topics.SEND_COMMAND_SERVICE, self._on_send_command,
            callback_group=cb)
        # Latched so a client's registration survives even if it was published
        # before the driver came up (or the driver restarts).
        self.shutdown_cmd_sub = self.create_subscription(
            String, Topics.SHUTDOWN_COMMAND_TOPIC, self._on_shutdown_command,
            latched, callback_group=cb)

        if self.backend in ("serial", "real", ""):
            source = f"{self.port} @ {self.baudrate} baud"
        elif self.backend in ("test", "dummy"):
            source = "in-memory test modem"
        elif self.backend == "unity":
            source = "smarcUnity acoustic transceiver"
        else:
            source = f"backend '{self.backend}'"
        self.get_logger().info(
            f"succorfish_driver_node ({self.profile}, {self.backend or 'serial'}) "
            f"bridging {source}  ->  RX:{Topics.RX_TOPIC}  TX:{Topics.TX_TOPIC}")

        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

    # ----- serial lifecycle -------------------------------------------------

    def _open_serial(self):
        for port in (self.port_override, self.port, self.port_fallback):
            if not port:
                continue
            try:
                ser = serial.Serial(port, self.baudrate, timeout=self.serial_timeout)
                if ser.is_open:
                    self.get_logger().info(f"Serial port {port} opened at {self.baudrate} baud")
                    return ser
            except serial.SerialException as exc:
                self.get_logger().warn(f"Could not open {port}: {exc}")
        return None

    def _make_backend(self):
        """Open the configured I/O source; returns a serial-like object or None.

        All backends expose the same ``in_waiting``/``read``/``write``/``close``
        slice of the pyserial interface, so the reader loop, service and shutdown
        paths are identical regardless of source.
        """
        if self.backend in ("serial", "real", ""):
            return self._open_serial()
        if self.backend in ("test", "dummy"):
            from succorfish_driver.backends import TestBackend
            gp = self.get_parameter
            return TestBackend(
                profile=self.profile,
                terminator=self.command_terminator,
                encoding=self.encoding,
                read_timeout=self.serial_timeout,
                range_m=gp("test.range_m").get_parameter_value().double_value,
                sound_velocity=gp("test.sound_velocity").get_parameter_value().double_value,
                drop_prob=gp("test.fault.drop_prob").get_parameter_value().double_value,
                garble_prob=gp("test.fault.garble_prob").get_parameter_value().double_value,
                logger=self.get_logger())
        if self.backend == "unity":
            from succorfish_driver.backends import UnityBackend, normalize_modem_id
            gp = self.get_parameter
            id_width = gp("unity.id_width").get_parameter_value().integer_value
            own_id = normalize_modem_id(
                gp("unity.own_modem_id").value, width=id_width)
            return UnityBackend(
                self,
                profile=self.profile,
                encoding=self.encoding,
                read_timeout=self.serial_timeout,
                terminator=self.command_terminator,
                write_topic=gp("unity.write_topic").get_parameter_value().string_value,
                read_topic=gp("unity.read_topic").get_parameter_value().string_value,
                own_modem_id=own_id,
                offset_us=gp("unity.offset_us").get_parameter_value().double_value,
                id_width=id_width)
        self.get_logger().error(
            f"Unknown backend '{self.backend}'; valid: serial, test, unity.")
        return None

    def _connect(self):
        try:
            ser = self._make_backend()
        except Exception as exc:  # noqa: BLE001 - bad params / missing deps
            self.get_logger().error(
                f"Could not open backend '{self.backend}': {exc}")
            ser = None
        if ser is not None:
            self._ser = ser
            self._set_connected(True)
        else:
            self._set_connected(False)
            self._stop_event.wait(self.reconnect_delay_s)

    def _on_disconnect(self, reason):
        self.get_logger().warn(f"Serial link lost ({reason}); reconnecting...")
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        self._set_connected(False)
        self._stop_event.wait(self.reconnect_delay_s)

    def _set_connected(self, value):
        if value != self._connected:
            self._connected = value
            self.status_pub.publish(Bool(data=value))

    # ----- reader thread ----------------------------------------------------

    def _reader_loop(self):
        while self._running:
            ser = self._ser
            if ser is None:
                self._connect()
                continue
            try:
                waiting = ser.in_waiting
                data = ser.read(waiting if waiting > 0 else 1)
            except Exception as exc:  # noqa: BLE001 - any serial error -> reconnect
                if self._running:
                    self._on_disconnect(f"read error: {exc}")
                continue
            if not data:
                continue
            for line in self._assembler.feed(data):
                self._publish_line(line)

    def _publish_line(self, line):
        msg = SerialLine()
        msg.stamp = self.get_clock().now().to_msg()
        msg.line = line
        self.rx_pub.publish(msg)
        with self._cond:
            self._seq += 1
            self._recent.append((self._seq, line))
            self._cond.notify_all()
        self.get_logger().debug(f"RX: {line!r}")

    # ----- writing ----------------------------------------------------------

    def _write(self, command, append_terminator):
        ser = self._ser
        if ser is None or not self._connected:
            return False, "serial port not connected"
        payload = command + (self.command_terminator if append_terminator else "")
        try:
            with self._write_lock:
                ser.write(payload.encode(self.encoding, "replace"))
            self.get_logger().debug(f"TX: {payload!r}")
            return True, ""
        except Exception as exc:  # noqa: BLE001 - any serial error -> reconnect
            self._on_disconnect(f"write error: {exc}")
            return False, f"write failed: {exc}"

    def _on_tx(self, msg):
        ok, err = self._write(msg.data, True)
        if not ok:
            self.get_logger().warn(f"TX dropped {msg.data!r}: {err}")

    def _on_shutdown_command(self, msg):
        self._shutdown_command = msg.data
        if msg.data:
            self.get_logger().info(
                f"Registered shutdown command {msg.data!r} (written to the wire "
                f"on graceful exit).")

    # ----- synchronous send-command service ---------------------------------

    def _on_send_command(self, request, response):
        with self._cond:
            start_seq = self._seq

        ok, err = self._write(request.command, request.append_terminator)
        if not ok:
            response.success = False
            response.message = err
            return response

        wants_reply = bool(request.expect_regex) or request.timeout > 0.0
        if not wants_reply:
            response.success = True
            response.message = "written"
            return response

        deadline = time.monotonic() + max(request.timeout, 0.0)
        collected = []
        seen_seq = start_seq
        matched = None

        with self._cond:
            while True:
                for seq, line in self._recent:
                    if seq > seen_seq:
                        collected.append(line)
                        seen_seq = seq
                if request.expect_regex:
                    matched = first_match(collected, request.expect_regex)
                remaining = deadline - time.monotonic()
                if matched is not None or remaining <= 0.0:
                    break
                self._cond.wait(timeout=remaining)

        response.lines = collected
        if request.expect_regex:
            response.success = matched is not None
            response.matched_line = matched or ""
            response.message = "matched" if matched else "timeout: no matching reply"
        else:
            response.success = True
            response.message = f"collected {len(collected)} line(s)"
        return response

    # ----- shutdown ---------------------------------------------------------

    def shutdown(self):
        self._running = False
        self._stop_event.set()  # wake the reader out of any reconnect wait
        try:
            if self._reader.is_alive():
                self._reader.join(timeout=2.0)
        except KeyboardInterrupt:
            # A second SIGINT during teardown: the reader is a daemon, so let go.
            pass
        # Last act while we still own an open port: replay the registered shutdown
        # command (e.g. an OWTT node's "$Y<id>W"), then let it drain before close.
        try:
            if self._shutdown_command and self._ser is not None and self._connected:
                ok, err = self._write(self._shutdown_command, True)
                if ok:
                    self.get_logger().info(
                        f"Wrote shutdown command {self._shutdown_command!r} on exit.")
                    try:
                        self._ser.flush()
                    except Exception:
                        pass
                    time.sleep(0.1)
                else:
                    self.get_logger().warn(f"Shutdown command not written: {err}")
        except Exception:
            pass
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
        finally:
            self._ser = None


def main(args=None):
    # Disable rclpy's default signal handlers and drive a manual spin loop so a
    # SIGINT/SIGTERM (and any escalating repeats from ros2 launch) just set a stop
    # flag instead of raising KeyboardInterrupt mid-teardown. This lets us close
    # the serial port and tear down cleanly without an ugly traceback.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SuccorfishDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    stop = {'stop': False}

    def _handler(signum, frame):
        stop['stop'] = True

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _handler)
        except (ValueError, OSError):
            pass

    try:
        while rclpy.ok() and not stop['stop']:
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
