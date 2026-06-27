"""Pluggable I/O backends for the succorfish driver.

The driver's value is its ROS contract (``rx``/``tx``/``connected``/
``send_command``/``shutdown_command``); *where the bytes come from* is a
separate concern. A backend is any object that quacks like the slice of
``serial.Serial`` the driver uses -- ``in_waiting``, ``read(size)``,
``write(bytes)``, ``close()`` -- so the driver's reader loop, line assembler,
service matching, status latching and shutdown replay all stay unchanged
regardless of the source.

Three backends ship:

* ``serial`` -- the real ``pyserial`` port (constructed in the node; this module
  only provides the two fakes).
* ``test`` (a.k.a. ``dummy``) -- an in-memory pretend modem. No hardware: it
  answers commands the way a modem would (``$P<id>`` -> ``#R<id>T<ticks>``,
  ``$Y<id><mode>`` -> ``#A<id>``), with optional fault injection. Good for
  exercising the behavioural loops with zero dependencies.
* ``unity`` -- a bridge to the smarcUnity acoustic ``Transceiver`` asset, which
  speaks ``smarc_msgs/StringStamped`` (opaque payload + ground-truth time of
  flight). This backend translates between that generic medium and the modem
  line dialect our client nodes expect, so the Unity asset stays generic and the
  firmware-specific knowledge lives here on the ROS side.

The ``unity`` backend keeps the Unity payload convention Luc used: the
``StringStamped.data`` field carries ``"<sender_modem_id>;<payload>"`` (physical
broadcast has no addressing, so the sender id rides along in the payload), and
range is recovered from ``time_received - time_sent`` rather than tick counting.
"""

import random
import re
import threading


# Succorfish two-way travel-time tick period (s); mirror of
# ``serial_ping_pkg.common.ping_protocol.TICK_PERIOD_S``. Kept local so the
# driver does not depend on the client package.
TICK_PERIOD_S = 0.00003125

_LINE_SPLIT = re.compile(r"\r\n|\n|\r")


def split_lines(text):
    """Split ``text`` into complete lines plus a trailing incomplete remainder.

    Returns ``(lines, remainder)``. ``remainder`` is whatever followed the last
    line terminator (empty if ``text`` ended on a terminator) and should be
    prepended to the next chunk.
    """
    parts = _LINE_SPLIT.split(text)
    remainder = parts.pop()
    return parts, remainder


def normalize_modem_id(value, width=3, default="001"):
    """Coerce a modem id into a canonical zero-padded string (e.g. ``'007'``).

    ``ros2 run``/``ros2 launch`` turn a numeric-looking value such as ``007``
    into the int ``7`` (or float), even when the parameter is declared as a
    string. This accepts ``str``/``int``/``float`` and returns the canonical
    ``width``-digit id, restoring any leading zero the numeric coercion dropped.
    Non-numeric values pass through unchanged.
    """
    if value is None or isinstance(value, bool):
        value = default
    if isinstance(value, float):
        value = int(round(value))
    s = str(value).strip()
    if s.endswith(".0") and s[:-2].isdigit():
        s = s[:-2]
    return s.zfill(width) if s.isdigit() else s


def _garble(line, rng):
    """Corrupt one character of ``line`` to model a noisy/garbled reply."""
    if not line:
        return line
    idx = rng.randrange(len(line))
    repl = rng.choice("0123456789#$TIRB?*")
    return line[:idx] + repl + line[idx + 1:]


# --------------------------------------------------------------------------- #
# Thread-safe byte channel (device -> host)                                   #
# --------------------------------------------------------------------------- #

class ByteChannel:
    """A thread-safe byte FIFO exposing the pyserial read interface.

    The fake backends push device->host bytes in with ``feed`` (from a ROS
    callback or a synchronous reply) while the driver's reader thread pulls them
    out via ``in_waiting``/``read`` exactly as it would a serial port.
    """

    def __init__(self, read_timeout=1.0):
        self._buf = bytearray()
        self._cv = threading.Condition()
        self._closed = False
        self._read_timeout = read_timeout

    @property
    def in_waiting(self):
        with self._cv:
            return len(self._buf)

    def read(self, size=1):
        """Return up to ``size`` bytes, blocking briefly when the buffer is empty."""
        with self._cv:
            if not self._buf and not self._closed:
                self._cv.wait(self._read_timeout)
            if size is None or size <= 0:
                size = len(self._buf)
            out = bytes(self._buf[:size])
            del self._buf[:size]
            return out

    def feed(self, data):
        """Push device->host bytes for the driver to read."""
        if not data:
            return
        with self._cv:
            self._buf.extend(data)
            self._cv.notify_all()

    @property
    def is_open(self):
        return not self._closed

    def close(self):
        with self._cv:
            self._closed = True
            self._cv.notify_all()


# --------------------------------------------------------------------------- #
# Modem models (the 'test' backend's brain)                                   #
# --------------------------------------------------------------------------- #

def succorfish_replies(command, range_m, sound_velocity):
    """Reply lines a Succorfish modem would emit for ``command`` (or ``[]``).

    Answers a ``$P<id>`` ping with ``#R<id>T<ticks>`` where ``ticks`` encodes
    ``range_m`` via the modem tick period, so the client recovers ``range_m``.
    """
    command = command.strip()
    if command.startswith("$P") and len(command) >= 5:
        modem_id = command[2:5]
        ticks = int(round(range_m / (TICK_PERIOD_S * sound_velocity)))
        return [f"#R{modem_id}T{ticks}"]
    return []


def teensy_replies(command):
    """Reply lines a Teensy OWTT front-end would emit for ``command`` (or ``[]``).

    Acknowledges a ``$Y<id><mode>`` config command with ``#A<id>`` (the line the
    OWTT nodes gate startup on).
    """
    command = command.strip()
    if command.startswith("$Y") and len(command) >= 6:
        return [f"#A{command[2:5]}"]
    return []


# --------------------------------------------------------------------------- #
# Test (dummy) backend                                                        #
# --------------------------------------------------------------------------- #

class TestBackend:
    """In-memory pretend modem: answers commands without any hardware."""

    __test__ = False  # not a pytest test class despite the 'Test' prefix

    def __init__(self, profile="succorfish", terminator="\r\n", encoding="utf-8",
                 read_timeout=1.0, range_m=100.0, sound_velocity=1500.0,
                 drop_prob=0.0, garble_prob=0.0, logger=None, rng=None):
        self._chan = ByteChannel(read_timeout)
        self._terminator = terminator
        self._encoding = encoding
        self._partial = ""
        self._profile = profile
        self._range_m = range_m
        self._sound_velocity = sound_velocity
        self._drop_prob = drop_prob
        self._garble_prob = garble_prob
        self._rng = rng or random.Random()
        self._log = logger

    # ----- pyserial-like interface -----
    @property
    def in_waiting(self):
        return self._chan.in_waiting

    def read(self, size=1):
        return self._chan.read(size)

    @property
    def is_open(self):
        return self._chan.is_open

    def close(self):
        self._chan.close()

    def write(self, data):
        text = self._partial + data.decode(self._encoding, "replace")
        lines, self._partial = split_lines(text)
        for line in lines:
            for reply in self._replies(line):
                reply = self._maybe_fault(reply)
                if reply is None:
                    continue
                self._chan.feed(
                    (reply + self._terminator).encode(self._encoding, "replace"))
        return len(data)

    # ----- internals -----
    def _replies(self, line):
        if self._profile == "teensy":
            return teensy_replies(line)
        return succorfish_replies(line, self._range_m, self._sound_velocity)

    def _maybe_fault(self, reply):
        if self._drop_prob and self._rng.random() < self._drop_prob:
            if self._log:
                self._log.warn(f"[test backend] dropped reply {reply!r}")
            return None
        if self._garble_prob and self._rng.random() < self._garble_prob:
            garbled = _garble(reply, self._rng)
            if self._log:
                self._log.warn(
                    f"[test backend] garbled reply {reply!r} -> {garbled!r}")
            return garbled
        return reply


# --------------------------------------------------------------------------- #
# Unity bridge: modem dialect <-> StringStamped + time-of-flight              #
# --------------------------------------------------------------------------- #

def unity_tx_payload(line):
    """Extract the over-the-water payload from an outbound modem command.

    Returns the opaque payload to broadcast through the Unity transceiver, or
    ``None`` if the command never reaches the water (Teensy-local config) or is
    not modelled by the one-way acoustic medium.
    """
    line = line.strip()
    if not line:
        return None
    if line.startswith("$Y"):
        # Teensy-local configuration (mode/address) -- never transmitted.
        return None
    if line.startswith("$P"):
        # Two-way ranging ping: a hardware round-trip concept with no analogue in
        # the one-way broadcast medium. The OWTT/TWTT broadcast loops are what the
        # sim models. Caller logs this.
        return None
    if line.startswith("$B") and len(line) >= 4:
        try:
            n = int(line[2:4])
            return line[4:4 + n]
        except ValueError:
            return line[2:]
    if line.startswith("$K"):
        # Telemetry update: the Teensy broadcasts it with a TEL: marker.
        return "TEL:" + line[2:]
    if line.startswith("$G"):
        # GPS update -> position broadcast payload (lat,lon).
        return line[2:]
    return line


def unity_rx_frames(data, tof_s, profile="succorfish", offset_us=0.0,
                    id_width=3, emit_delta=None):
    """Synthesize the modem line(s) a received Unity packet would produce.

    ``data`` is the ``StringStamped`` payload (``"<sender_id>;<payload>"`` by
    convention); ``tof_s`` is the ground-truth time of flight. Returns a list of
    modem lines: a ``#B<id><nn><payload>`` broadcast and, for OWTT, the paired
    ``#I<delta_us>`` timing line that lets a follower compute range.
    """
    if emit_delta is None:
        emit_delta = profile == "teensy"
    if ";" in data:
        sender_id, payload = data.split(";", 1)
    else:
        sender_id, payload = data, ""
    sid = str(sender_id).strip().zfill(id_width)[-id_width:]
    frames = [f"#B{sid}{len(payload):02d}{payload}"]
    if emit_delta:
        delta_us = max(tof_s, 0.0) * 1e6 + offset_us
        frames.append(f"#I{delta_us:.0f}")
    return frames


class UnityBackend:
    """Bridge the smarcUnity acoustic Transceiver to the modem line dialect.

    Publishes outbound payloads as ``StringStamped`` on ``write_topic`` and turns
    inbound ``StringStamped`` (payload + time of flight) on ``read_topic`` into
    the ``#B``/``#I`` modem lines the client nodes parse.
    """

    def __init__(self, node, profile="succorfish", encoding="utf-8",
                 read_timeout=1.0, write_topic="acoustic/write",
                 read_topic="acoustic/read", own_modem_id="001",
                 offset_us=0.0, emit_delta=None, id_width=3, terminator="\r\n"):
        from smarc_msgs.msg import StringStamped  # lazy: only needed for unity
        self._StringStamped = StringStamped
        self._node = node
        self._chan = ByteChannel(read_timeout)
        self._encoding = encoding
        self._terminator = terminator
        self._partial = ""
        self._profile = profile
        self._own_id = str(own_modem_id)
        self._offset_us = float(offset_us)
        self._id_width = id_width
        self._emit_delta = (profile == "teensy") if emit_delta is None else emit_delta
        self._log = node.get_logger()
        self._warned_ping = False
        self._pub = node.create_publisher(StringStamped, write_topic, 10)
        self._sub = node.create_subscription(
            StringStamped, read_topic, self._on_acoustic, 10)

    # ----- pyserial-like interface -----
    @property
    def in_waiting(self):
        return self._chan.in_waiting

    def read(self, size=1):
        return self._chan.read(size)

    @property
    def is_open(self):
        return self._chan.is_open

    def close(self):
        self._chan.close()
        try:
            self._node.destroy_publisher(self._pub)
            self._node.destroy_subscription(self._sub)
        except Exception:
            pass

    def write(self, data):
        text = self._partial + data.decode(self._encoding, "replace")
        lines, self._partial = split_lines(text)
        for line in lines:
            # Local Teensy control plane (e.g. $Y config -> #A ack) is answered
            # by the front-end itself and never goes on the water. We emulate it
            # locally; without it a transmitter-mode node never sees its
            # #A<id> confirmation and refuses to broadcast.
            for reply in self._local_replies(line):
                self._chan.feed(
                    (reply + self._terminator).encode(self._encoding, "replace"))
            if line.strip().startswith("$P") and not self._warned_ping:
                self._warned_ping = True
                self._log.warn(
                    "Two-way $P ranging is not modelled by the unity bridge "
                    "(it has no analogue in the one-way acoustic medium); use "
                    "the broadcast/OWTT loops for sim ranging.")
            payload = unity_tx_payload(line)
            if payload is None:
                continue
            msg = self._StringStamped()
            msg.data = f"{self._own_id};{payload}"
            now = self._node.get_clock().now().to_msg()
            msg.time_sent = now
            msg.time_received = now
            self._pub.publish(msg)
        return len(data)

    def _local_replies(self, line):
        """Lines the front-end would answer locally (not over the water)."""
        if self._profile == "teensy":
            return teensy_replies(line)
        return []

    def _on_acoustic(self, msg):
        # Unity's AcousticReceiver_Pub republishes its last message at a fixed
        # rate even when nothing was actually received; that idle message has
        # empty data and zero-valued timestamps. Drop it so we don't fabricate
        # phantom receptions (and spurious #I0 deltas) out of silence.
        if not msg.data:
            return
        ts = msg.time_sent.sec + msg.time_sent.nanosec
        tr = msg.time_received.sec + msg.time_received.nanosec
        if ts == 0 and tr == 0:
            return
        tof_s = self._tof_seconds(msg)
        frames = unity_rx_frames(
            msg.data, tof_s, profile=self._profile, offset_us=self._offset_us,
            id_width=self._id_width, emit_delta=self._emit_delta)
        for frame in frames:
            self._chan.feed(
                (frame + self._terminator).encode(self._encoding, "replace"))

    @staticmethod
    def _tof_seconds(msg):
        ts = msg.time_sent.sec + msg.time_sent.nanosec * 1e-9
        tr = msg.time_received.sec + msg.time_received.nanosec * 1e-9
        return tr - ts
