# Centralized topic and service names for the succorfish_driver serial bridge.
# Import in Python as `from succorfish_msgs.msg import Topics` and reference
# Topics.RX_TOPIC, etc. Names are relative; nodes namespace them at runtime.

string RX_TOPIC = "succorfish/rx"                       # succorfish_msgs/SerialLine - every inbound line (driver -> clients)
string TX_TOPIC = "succorfish/tx"                       # std_msgs/String           - raw outbound command (clients -> driver)
string STATUS_TOPIC = "succorfish/connected"            # std_msgs/Bool             - link up/down (driver -> clients)
string SEND_COMMAND_SERVICE = "succorfish/send_command" # succorfish_msgs/SendCommand - synchronous request/response
string SHUTDOWN_COMMAND_TOPIC = "succorfish/shutdown_command" # std_msgs/String     - latched; client registers a command the driver writes on graceful exit
