# succorfish_driver

A transparent ROS 2 **serial bridge** for the Succorfish Delphis modem and the
Teensy 4.1 OWTT front-end. One node exclusively owns a single serial port and
exposes it to the rest of the ROS graph as a protocol-agnostic line pipe, so
other nodes never open the port themselves — they just publish/subscribe topics
(or call a service).

This repo is normally consumed as a git submodule of the `serial_ping` meta-repo,
but it builds standalone in any ROS 2 (Jazzy) workspace.

## Packages

| Package | Build type | Purpose |
|---------|-----------|---------|
| `succorfish_driver` | `ament_python` | The bridge node (`succorfish_driver_node`). |
| `succorfish_msgs` | `ament_cmake` / `rosidl` | `Topics.msg` (centralized names), `SerialLine.msg`, `SendCommand.srv`. |

## Architecture

```
            RX_TOPIC (SerialLine)            +-----------------+
   clients <----------------------------     |                 |
                                             | succorfish_     |  serial   +--------+
   clients ---------------------------->     | driver_node     | <-------> | modem  |
            TX_TOPIC (std_msgs/String)       | (owns the port) |           +--------+
                                             |                 |
   clients <===== SendCommand srv =====>     +-----------------+
            STATUS_TOPIC (std_msgs/Bool, latched)
```

The driver does **not** understand the modem protocol (`$P`, `$B`, `#R...T...`,
`#I`, ...). Framing/parsing stays in the client nodes (e.g. the helper modules in
`serial_ping_pkg`). The driver only moves whole lines in and out.

The serial link itself is pluggable: the `backend` parameter swaps the real port
for an in-memory pretend modem or a smarcUnity bridge without the client nodes
noticing (see [Backends](#backends-where-the-bytes-come-from)).

## ROS interface

Names are constants in `succorfish_msgs/msg/Topics.msg` (import as
`from succorfish_msgs.msg import Topics`). Defaults (relative, namespaced at runtime):

| Constant | Name | Type | Direction |
|----------|------|------|-----------|
| `RX_TOPIC` | `succorfish/rx` | `succorfish_msgs/SerialLine` | driver -> clients, one msg per inbound line |
| `TX_TOPIC` | `succorfish/tx` | `std_msgs/String` | clients -> driver, raw command written to the port |
| `STATUS_TOPIC` | `succorfish/connected` | `std_msgs/Bool` (latched) | driver -> clients, link up/down |
| `SEND_COMMAND_SERVICE` | `succorfish/send_command` | `succorfish_msgs/SendCommand` | synchronous request/response |
| `SHUTDOWN_COMMAND_TOPIC` | `succorfish/shutdown_command` | `std_msgs/String` (latched) | clients -> driver, command to write on graceful exit |

### Shutdown command (on-exit hook)

The driver stays protocol-agnostic but can guarantee a final command reaches the
wire when *it* shuts down. A client publishes an opaque string (latched) to
`SHUTDOWN_COMMAND_TOPIC` **once at startup**; the driver remembers the last value
and writes it to the port — then flushes and closes — on a graceful exit. Because
the registration happens up front and the driver is the last holder of the open
port, the command is delivered regardless of shutdown ordering (no race even when
the driver and the client are torn down together).

The meaning of the string lives entirely with the registering node. For example,
OWTT-capable nodes register `$Y<id>W` (Teensy wire mode) so the modem is always
returned to wire mode; the plain Succorfish profile registers nothing and the
driver writes nothing on exit. Clients using `serial_ping_pkg` do this via
`DriverClient.register_shutdown_command(...)`.

### SendCommand service

Write a command and (optionally) wait for a reply line — useful for the blocking
request/response interactions (e.g. a ping):

```
string command                 # raw command to write
bool   append_terminator true  # append the configured terminator (\r\n)
string expect_regex            # if non-empty, wait for a line matching this regex
float64 timeout                # seconds to wait (<=0 = write and return)
---
bool     success               # written, and (if expect_regex) matched before timeout
string   matched_line          # first line matching expect_regex
string[] lines                 # all lines received during the wait
string   message               # status / error detail
```

## Hardware profiles (the two configs)

Two bundled config files, one per hardware stack:

| File | Port | Baud | Talks to |
|------|------|------|----------|
| `config/succorfish.yaml` | `/dev/ttyUSB0` (fallback `ttyUSB1`) | 9600 | Succorfish Delphis modem directly |
| `config/teensy.yaml` | `/dev/ttyACM0` (fallback `ttyACM1`) | 115200 | Teensy 4.1 OWTT front-end |

Parameters: `serial.port`, `serial.port_fallback`, `serial.baudrate`,
`serial.timeout`, `command_terminator`, `encoding`, `reconnect_delay_s`,
`recent_lines_buffer`, `profile`, `backend`. The node reconnects automatically
on link loss.

## Backends (where the bytes come from)

The driver's value is its ROS contract, not the wire underneath it. The
`backend` parameter selects the byte source; every backend exposes the same
interface to the driver, so RX/TX/SendCommand/shutdown behave identically no
matter which one is active:

| `backend` | Needs hardware? | What it is |
|-----------|-----------------|------------|
| `serial` (default) | yes | The real `pyserial` port (a Succorfish modem or Teensy). |
| `test` (alias `dummy`) | no | An in-memory **pretend modem** that answers commands the way the firmware would. |
| `unity` | no (needs `smarc_msgs`) | A bridge to the **smarcUnity** acoustic `Transceiver` asset (`StringStamped` + time-of-flight). |

This is what lets you swap the Succorfish for anything that speaks the same
language (a different physical modem, a simulator, a stub) by changing only the
driver — the client nodes never notice.

### `test` backend — pretend modem

Answers per the active `profile`, deterministically and with no port:

- `succorfish`: `$P<id>` → `#R<id>T<ticks>`, where `ticks` encodes
  `test.range_m` via the modem tick period (so the client recovers that range).
- `teensy`: `$Y<id><mode>` → `#A<id>` (the config ack the OWTT nodes gate on).

Optional fault injection stress-tests client robustness: `test.fault.drop_prob`
drops replies, `test.fault.garble_prob` corrupts them (both logged). Knobs:
`test.range_m`, `test.sound_velocity`, `test.fault.drop_prob`,
`test.fault.garble_prob`.

### `unity` backend — smarcUnity acoustic bridge

The Unity `Transceiver` is intentionally generic: it broadcasts an opaque string
payload and reports ground-truth time-of-flight via `smarc_msgs/StringStamped`
(`data`, `time_sent`, `time_received`). Rather than teach Unity the modem
protocol, this backend keeps the firmware-specific knowledge on the ROS side and
translates both ways:

- **TX** (client → water): the outbound command's payload is extracted
  (`$B<nn><data>` → `data`, `$K<p>` → `TEL:<p>`, `$G<lat>,<lon>` → position;
  `$Y` config and `$P` two-way pings are not transmitted) and published as
  `StringStamped.data = "<own_modem_id>;<payload>"` on `unity.write_topic`.
- **RX** (water → client): an incoming `StringStamped` becomes the modem lines a
  client expects — `#B<sender_id><nn><payload>` plus, for the OWTT (`teensy`)
  profile, a paired `#I<delta_us>` timing line with `delta = (time_received -
  time_sent) + unity.offset_us`, which is exactly what a follower needs to derive
  range.

Knobs: `unity.write_topic` (default `acoustic/write`), `unity.read_topic`
(default `acoustic/read`), `unity.own_modem_id`, `unity.offset_us`,
`unity.id_width`. The Unity payload convention (`"<id>;<payload>"` + ToF) matches
the existing TUPER sim assets, so the asset stays unmodified. Two-way `$P`
ranging has no analogue in the one-way acoustic medium and is not modelled — use
the broadcast/OWTT loops for ranging in sim.

## Run

```bash
# Succorfish modem profile (default)
ros2 launch succorfish_driver succorfish_driver.launch

# Teensy front-end profile
ros2 launch succorfish_driver succorfish_driver.launch profile:=teensy

# Namespace the node per robot (e.g. 'sam')
ros2 launch succorfish_driver succorfish_driver.launch profile:=teensy namespace:=sam

# Bring your own params file (overrides the profile file)
ros2 launch succorfish_driver succorfish_driver.launch config_file:=/abs/path/to/serial.yaml

# No hardware: in-memory pretend modem
ros2 launch succorfish_driver succorfish_driver.launch backend:=test

# Bridge the smarcUnity acoustic transceiver (per-robot namespace + modem id)
ros2 launch succorfish_driver succorfish_driver.launch \
    profile:=teensy backend:=unity namespace:=lolo unity_own_modem_id:=101

# Or run the node directly
ros2 run succorfish_driver succorfish_driver_node --ros-args \
    --params-file $(ros2 pkg prefix succorfish_driver)/share/succorfish_driver/config/succorfish.yaml
```

Quick manual check once running:

```bash
ros2 topic echo /succorfish/rx                       # watch inbound lines
ros2 topic pub --once /succorfish/tx std_msgs/String "{data: '$P001'}"   # send a command
ros2 service call /succorfish/send_command succorfish_msgs/srv/SendCommand \
    "{command: '$P001', append_terminator: true, expect_regex: 'T\\d+', timeout: 3.0}"
```

## Tests

```bash
colcon test --packages-select succorfish_driver
# or directly (after sourcing the workspace install):
python3 -m pytest test/ -q
```

`test/test_line_assembler.py` covers the pure line-assembly logic; the pty-based
`test/test_driver_integration.py` exercises RX/TX/SendCommand and the on-exit
shutdown-command write over a virtual serial port. `test/test_backends.py`
unit-tests the `test`/`unity` backend logic (no ROS), and
`test/test_backend_modes.py` runs the `test` and `unity` backends end-to-end over
a real rclpy graph (no hardware).

## Using it from other nodes

All `serial_ping_pkg` nodes now talk to the driver instead of opening a port. The
`serial_ping_pkg.common.driver_client.DriverClient` helper wraps the interface:

1. Construct `DriverClient(node, on_line=...)` — it sets up the RX subscription,
   TX publisher, status subscription, and the `SendCommand` client.
2. Handle inbound lines in your `on_line` callback (run the existing parser on
   `line.line`).
3. Publish outbound commands with `client.write(...)`, or use `client.request(...)`
   for blocking request/response (ping).
4. Optionally call `client.register_shutdown_command(...)` at startup to have the
   driver replay a final command on its own exit (see above).

## License

MIT — Shekhar Devm Upadhyay (sdup@kth.se).
