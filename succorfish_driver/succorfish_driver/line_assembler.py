"""Pure, hardware-free helpers for turning a raw serial byte stream into lines.

Kept free of ROS and pyserial so it can be unit tested directly, mirroring the
``*_protocol`` helper convention used elsewhere in ``serial_ping_pkg``.
"""

import re


class LineAssembler:
    """Accumulates raw bytes and emits complete, decoded, stripped lines.

    Serial reads arrive in arbitrary chunks, so a line may be split across
    several reads or several lines may arrive in one read. ``feed`` buffers the
    bytes and yields each complete line (terminator removed). Both ``\\n`` and
    ``\\r\\n`` terminators are handled; surrounding whitespace is stripped.
    """

    def __init__(self, encoding="utf-8", errors="replace", keep_empty=False):
        self._buf = bytearray()
        self.encoding = encoding
        self.errors = errors
        self.keep_empty = keep_empty

    def feed(self, chunk):
        """Feed raw bytes and yield every complete line they complete."""
        if not chunk:
            return
        self._buf.extend(chunk)
        while True:
            idx = self._buf.find(b"\n")
            if idx == -1:
                break
            raw = bytes(self._buf[:idx])
            del self._buf[: idx + 1]
            line = raw.decode(self.encoding, self.errors).strip()
            if line or self.keep_empty:
                yield line

    def flush(self):
        """Return any buffered bytes as a final line and clear the buffer.

        Useful when the stream closes without a trailing terminator. Returns
        ``None`` if there is nothing (or only whitespace) buffered.
        """
        if not self._buf:
            return None
        line = bytes(self._buf).decode(self.encoding, self.errors).strip()
        self._buf.clear()
        if line or self.keep_empty:
            return line
        return None

    @property
    def pending(self):
        """Bytes buffered so far that do not yet form a complete line."""
        return bytes(self._buf)


def first_match(lines, pattern):
    """Return the first line matching ``pattern`` (regex search), else ``None``.

    An empty/None pattern never matches (returns ``None``).
    """
    if not pattern:
        return None
    regex = re.compile(pattern)
    for line in lines:
        if regex.search(line):
            return line
    return None
