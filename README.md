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

## ROS interface

Names are constants in `succorfish_msgs/msg/Topics.msg` (import as
`from succorfish_msgs.msg import Topics`). Defaults (relative, namespaced at runtime):

| Constant | Name | Type | Direction |
|----------|------|------|-----------|
| `RX_TOPIC` | `succorfish/rx` | `succorfish_msgs/SerialLine` | driver -> clients, one msg per inbound line |
| `TX_TOPIC` | `succorfish/tx` | `std_msgs/String` | clients -> driver, raw command written to the port |
| `STATUS_TOPIC` | `succorfish/connected` | `std_msgs/Bool` (latched) | driver -> clients, link up/down |
| `SEND_COMMAND_SERVICE` | `succorfish/send_command` | `succorfish_msgs/SendCommand` | synchronous request/response |

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
| `config/succorfish_modem.yaml` | `/dev/ttyUSB0` (fallback `ttyUSB1`) | 9600 | Succorfish Delphis modem directly |
| `config/teensy.yaml` | `/dev/ttyACM0` (fallback `ttyACM1`) | 115200 | Teensy 4.1 OWTT front-end |

Parameters: `serial.port`, `serial.port_fallback`, `serial.baudrate`,
`serial.timeout`, `command_terminator`, `encoding`, `reconnect_delay_s`,
`recent_lines_buffer`, `profile`. The node reconnects automatically on link loss.

## Run

```bash
# Succorfish modem profile (default)
ros2 launch succorfish_driver succorfish_driver.launch.py

# Teensy front-end profile
ros2 launch succorfish_driver succorfish_driver.launch.py profile:=teensy

# Override port/baud and namespace the node (e.g. per robot)
ros2 launch succorfish_driver succorfish_driver.launch.py \
    profile:=succorfish port:=/dev/ttyUSB2 baudrate:=9600 namespace:=sam

# Bring your own params file
ros2 launch succorfish_driver succorfish_driver.launch.py config_file:=/abs/path/to/serial.yaml

# Or run the node directly
ros2 run succorfish_driver succorfish_driver_node --ros-args \
    --params-file $(ros2 pkg prefix succorfish_driver)/share/succorfish_driver/config/succorfish_modem.yaml
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
`test/test_driver_integration.py` exercises RX/TX/SendCommand over a virtual
serial port.

## Using it from other nodes (future migration)

Existing `serial_ping_pkg` nodes currently open the port directly via
`init_serial`. To migrate a node to the driver:

1. Drop the `serial.Serial` / `init_serial` call.
2. Subscribe `RX_TOPIC` (`SerialLine`) and run the existing parser helper on
   `msg.line`.
3. Publish outbound commands to `TX_TOPIC` (`std_msgs/String`), or use the
   `SendCommand` service for blocking request/response (ping).

This migration is intentionally **not** done yet — the driver ships first.

## License

MIT — Shekhar Devm Upadhyay (sdup@kth.se).
