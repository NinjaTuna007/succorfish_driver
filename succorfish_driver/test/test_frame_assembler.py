"""Unit tests for NM3 length-prefixed frame assembly (no ROS, no hardware)."""

from succorfish_driver.frame_assembler import FrameAssembler


def test_newline_line_still_works():
    fa = FrameAssembler()
    assert list(fa.feed(b'#R001T1234\r\n')) == [(b'#R001T1234', '#R001T1234')]


def test_ascii_broadcast_also_emits_text():
    fa = FrameAssembler()
    raw = b'#B00705Hello'
    assert list(fa.feed(raw + b'\r\n')) == [(raw, '#B00705Hello')]


def test_binary_broadcast_does_not_split_on_newline_in_payload():
    fa = FrameAssembler()
    payload = b'He\nlo'  # 5 bytes, includes LF
    frame = b'#B00705' + payload
    assert list(fa.feed(frame + b'\r\n')) == [(frame, None)]
    assert fa.pending == b''


def test_binary_unicast_with_trailer():
    fa = FrameAssembler()
    payload = bytes([0x00, 0x01, 0x0A, 0xFF])
    frame = b'#U04' + payload + b'Q56D+000'
    got = list(fa.feed(frame + b'\r\n'))
    assert got == [(frame, None)]


def test_split_across_chunks():
    fa = FrameAssembler()
    payload = b'A\nB\nC'  # 5 bytes
    assert list(fa.feed(b'#B11105A')) == []
    assert list(fa.feed(b'\nB\nC\r')) == []
    assert list(fa.feed(b'\n')) == [(b'#B11105A\nB\nC', None)]


def test_text_after_binary():
    fa = FrameAssembler()
    payload = b'\x00\x00'
    chunk = b'#U02' + payload + b'\r\n#I12345\r\n'
    assert list(fa.feed(chunk)) == [
        (b'#U02\x00\x00', None),
        (b'#I12345', '#I12345'),
    ]


def test_non_data_hash_line_is_still_newline():
    fa = FrameAssembler()
    assert list(fa.feed(b'#E,Y,BUSY\r\n')) == [(b'#E,Y,BUSY', '#E,Y,BUSY')]


def test_broadcast_header_lookalike_without_digits_is_newline():
    """``#BUSY`` starts with ``#B`` but is not a 3-digit-id data frame."""
    fa = FrameAssembler()
    assert list(fa.feed(b'#BUSY\r\n')) == [(b'#BUSY', '#BUSY')]


def test_ascii_broadcast_keeps_trailer_on_both_outputs():
    fa = FrameAssembler()
    raw = b'#B00705HelloQ56D+000'
    assert list(fa.feed(raw + b'\r\n')) == [(raw, raw.decode('ascii'))]
