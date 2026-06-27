"""Pure unit tests for the pluggable driver backends (no ROS, no hardware).

Covers the in-memory byte channel, the 'test' modem model (Succorfish ping and
Teensy config), fault injection, and the 'unity' codec (command -> payload and
StringStamped+ToF -> modem lines).
"""

import random

from succorfish_driver.backends import (
    ByteChannel,
    TICK_PERIOD_S,
    TestBackend,
    normalize_modem_id,
    split_lines,
    succorfish_replies,
    teensy_replies,
    unity_rx_frames,
    unity_tx_payload,
)


# --------------------------------------------------------------------------- #
# normalize_modem_id                                                          #
# --------------------------------------------------------------------------- #

def test_normalize_modem_id_restores_leading_zeros():
    # ros2 coerces 007 -> int 7 / float 7.0; we must recover "007".
    assert normalize_modem_id(7) == "007"
    assert normalize_modem_id(7.0) == "007"
    assert normalize_modem_id("007") == "007"
    assert normalize_modem_id("7", width=4) == "0007"
    assert normalize_modem_id(None) == "001"


# --------------------------------------------------------------------------- #
# split_lines / ByteChannel                                                   #
# --------------------------------------------------------------------------- #

def test_split_lines_keeps_incomplete_remainder():
    lines, remainder = split_lines("$P001\r\n$Y101R\r\n$par")
    assert lines == ["$P001", "$Y101R"]
    assert remainder == "$par"


def test_split_lines_complete_leaves_empty_remainder():
    lines, remainder = split_lines("a\r\nb\r\n")
    assert lines == ["a", "b"]
    assert remainder == ""


def test_byte_channel_feed_read_close():
    chan = ByteChannel(read_timeout=0.05)
    chan.feed(b"hello")
    assert chan.in_waiting == 5
    assert chan.read(3) == b"hel"
    assert chan.read(10) == b"lo"
    # Empty + closed returns immediately without blocking.
    chan.close()
    assert chan.read(1) == b""
    assert chan.is_open is False


# --------------------------------------------------------------------------- #
# Modem models                                                                #
# --------------------------------------------------------------------------- #

def test_succorfish_ping_reply_encodes_range():
    # 1500 m at 1500 m/s -> ticks = 1500 / (TICK_PERIOD_S * 1500) = 32000.
    replies = succorfish_replies("$P007", range_m=1500.0, sound_velocity=1500.0)
    assert replies == ["#R007T32000"]
    # And the client-side conversion recovers the range.
    ticks = int(replies[0].split("T")[1])
    assert abs(TICK_PERIOD_S * 1500.0 * ticks - 1500.0) < 1e-6


def test_succorfish_ignores_non_ping():
    assert succorfish_replies("$Y101R", 100.0, 1500.0) == []


def test_teensy_config_is_acked():
    assert teensy_replies("$Y101R") == ["#A101"]
    assert teensy_replies("$Y042T0074s") == ["#A042"]
    assert teensy_replies("$G59.0,18.0") == []


# --------------------------------------------------------------------------- #
# TestBackend (in-memory pretend modem)                                       #
# --------------------------------------------------------------------------- #

def test_test_backend_succorfish_ping_roundtrip():
    be = TestBackend(profile="succorfish", read_timeout=0.1,
                     range_m=100.0, sound_velocity=1500.0)
    be.write(b"$P007\r\n")
    line = be.read(be.in_waiting).decode()
    assert line.strip() == "#R007T2133"  # round(100 / 0.046875)


def test_test_backend_teensy_config_ack():
    be = TestBackend(profile="teensy", read_timeout=0.1)
    be.write(b"$Y101R\r\n")
    assert be.read(be.in_waiting).decode().strip() == "#A101"


def test_test_backend_drop_fault_suppresses_reply():
    be = TestBackend(profile="succorfish", read_timeout=0.05, drop_prob=1.0)
    be.write(b"$P007\r\n")
    assert be.in_waiting == 0


def test_test_backend_garble_fault_changes_reply():
    be = TestBackend(profile="succorfish", read_timeout=0.05, garble_prob=1.0,
                     rng=random.Random(1))
    be.write(b"$P007\r\n")
    line = be.read(be.in_waiting).decode().strip()
    assert line != "#R007T2133" and line  # corrupted but non-empty


# --------------------------------------------------------------------------- #
# Unity codec                                                                 #
# --------------------------------------------------------------------------- #

def test_unity_tx_extracts_broadcast_payload():
    assert unity_tx_payload("$B0959.0,18.0") == "59.0,18.0"


def test_unity_tx_marks_telemetry():
    assert unity_tx_payload("$Kfoo=1") == "TEL:foo=1"


def test_unity_tx_gps_becomes_position():
    assert unity_tx_payload("$G59.0,18.0") == "59.0,18.0"


def test_unity_tx_drops_local_config_and_ping():
    assert unity_tx_payload("$Y101R") is None
    assert unity_tx_payload("$P007") is None


def test_unity_rx_succorfish_no_delta():
    frames = unity_rx_frames("042;59.0,18.0", tof_s=0.05, profile="succorfish")
    assert frames == ["#B04209" "59.0,18.0"]


def test_unity_rx_teensy_emits_owtt_delta():
    frames = unity_rx_frames("042;59.0,18.0", tof_s=0.05, profile="teensy")
    assert frames[0] == "#B0420959.0,18.0"
    assert frames[1] == "#I50000"  # 0.05 s -> 50000 us


def test_unity_rx_delta_applies_offset():
    frames = unity_rx_frames("007;hi", tof_s=0.01, profile="teensy",
                             offset_us=250.0)
    assert frames[1] == "#I10250"  # 0.01 s -> 10000 us + 250 offset
