"""Assemble UART bytes into complete frames, including NM3 length-prefixed data.

Text commands and acks are newline-delimited. Broadcast/unicast *data* frames
are not: ``#B<aaa><nn><data>...`` / ``#U<nn><data>...`` carry ``nn`` payload
bytes that may include CR/LF. The length field is consumed first, then the
optional ASCII trailer (LQI / Doppler / timestamp) is read until CRLF.

Kept free of ROS so it can be unit-tested directly.
"""


_PRINTABLE = frozenset(range(0x20, 0x7F))


def _digits2(buf, offset):
    """Return the two-digit ASCII length at ``offset``, or None."""
    if offset + 2 > len(buf):
        return None
    a, b = buf[offset], buf[offset + 1]
    if 0x30 <= a <= 0x39 and 0x30 <= b <= 0x39:
        return (a - 0x30) * 10 + (b - 0x30)
    return None


def _nm3_data_header(buf):
    """If ``buf`` starts as a length-prefixed NM3 data frame, return layout.

    Returns ``(payload_start, nbytes)`` or ``None`` if this is not a data
    header (or the header is still incomplete). ``None`` with a short buffer
    that *might* still become a header is signalled by raising nothing —
    callers distinguish "need more header bytes" vs "not a data frame".
    """
    if not buf:
        return None
    if buf[0] != 0x23:  # '#'
        return None
    if len(buf) < 2:
        return 'incomplete'
    if buf[1] == 0x42:  # 'B'  #B<aaa><nn>
        if len(buf) < 7:
            return 'incomplete'
        if not (all(0x30 <= buf[i] <= 0x39 for i in range(2, 7))):
            return None
        n = _digits2(buf, 5)
        return (7, n)
    if buf[1] == 0x55:  # 'U'  #U<nn>
        if len(buf) < 4:
            return 'incomplete'
        n = _digits2(buf, 2)
        if n is None:
            return None
        return (4, n)
    return None


def payload_is_printable(frame, payload_start, nbytes):
    """True when the NM3 data payload is 7-bit printable ASCII."""
    chunk = frame[payload_start:payload_start + nbytes]
    return len(chunk) == nbytes and all(b in _PRINTABLE for b in chunk)


class FrameAssembler:
    """Accumulates raw UART bytes and yields complete frames (CRLF stripped).

    Each yielded item is ``(raw_bytes, text_line_or_none)``. ``text_line`` is
    a UTF-8 string suitable for ``SerialLine`` when the frame is a normal
    newline record, or an NM3 data frame whose payload is printable. Binary
    data frames yield ``text_line is None`` so existing string subscribers
    do not see a split or mojibake line.
    """

    def __init__(self, encoding="utf-8", errors="replace"):
        self._buf = bytearray()
        self.encoding = encoding
        self.errors = errors

    def feed(self, chunk):
        if not chunk:
            return
        self._buf.extend(chunk)
        while True:
            frame = self._pop_frame()
            if frame is None:
                break
            yield frame

    def _pop_frame(self):
        buf = self._buf
        if not buf:
            return None

        header = _nm3_data_header(buf)
        if header == 'incomplete':
            return None
        if header is not None:
            payload_start, nbytes = header
            # Payload, then optional ASCII trailer, then CRLF.
            after_payload = payload_start + nbytes
            if len(buf) < after_payload:
                return None
            nl = bytes(buf[after_payload:]).find(b'\n')
            if nl == -1:
                return None
            end = after_payload + nl
            raw = bytes(buf[:end])
            if raw.endswith(b'\r'):
                raw = raw[:-1]
            del buf[:end + 1]
            text = None
            if payload_is_printable(raw, payload_start, nbytes):
                text = raw.decode(self.encoding, self.errors).strip()
            return raw, text

        nl = buf.find(b'\n')
        if nl == -1:
            return None
        raw = bytes(buf[:nl])
        if raw.endswith(b'\r'):
            raw = raw[:-1]
        del buf[:nl + 1]
        if not raw:
            return self._pop_frame()
        text = raw.decode(self.encoding, self.errors).strip()
        if not text:
            return self._pop_frame()
        return raw, text

    @property
    def pending(self):
        return bytes(self._buf)
