# A complete serial frame as raw bytes.
#
# On TX (succorfish/tx_bytes): written to the UART as-is. No terminator is
# appended (NM3 commands do not require one). Include CRLF in data if you want it.
# On RX (succorfish/rx_bytes): one complete frame with the trailing CRLF stripped.

builtin_interfaces/Time stamp
uint8[] data
