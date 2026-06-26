"""Unit tests for the pure line-assembly helpers (no ROS, no hardware)."""

from succorfish_driver.line_assembler import LineAssembler, first_match


def test_single_complete_line():
    la = LineAssembler()
    assert list(la.feed(b"hello\n")) == ["hello"]


def test_crlf_is_stripped():
    la = LineAssembler()
    assert list(la.feed(b"$P001\r\n")) == ["$P001"]


def test_multiple_lines_in_one_chunk():
    la = LineAssembler()
    assert list(la.feed(b"a\r\nb\r\nc\r\n")) == ["a", "b", "c"]


def test_line_split_across_chunks():
    la = LineAssembler()
    assert list(la.feed(b"#R001T")) == []
    assert list(la.feed(b"1234")) == []
    assert list(la.feed(b"\r\n")) == ["#R001T1234"]


def test_partial_remainder_is_buffered():
    la = LineAssembler()
    assert list(la.feed(b"first\npart")) == ["first"]
    assert la.pending == b"part"


def test_empty_lines_skipped_by_default():
    la = LineAssembler()
    assert list(la.feed(b"\r\n\r\nx\r\n")) == ["x"]


def test_empty_lines_kept_when_requested():
    la = LineAssembler(keep_empty=True)
    assert list(la.feed(b"\n\nx\n")) == ["", "", "x"]


def test_flush_returns_trailing_partial():
    la = LineAssembler()
    list(la.feed(b"tail"))
    assert la.flush() == "tail"
    assert la.flush() is None


def test_first_match_finds_earliest():
    lines = ["#TO", "#R001T10", "#R002T20"]
    assert first_match(lines, r"T\d+") == "#R001T10"


def test_first_match_no_pattern_returns_none():
    assert first_match(["anything"], "") is None
    assert first_match(["anything"], None) is None


def test_first_match_no_hit_returns_none():
    assert first_match(["#TO", "junk"], r"^#R\d{3}T\d+$") is None
