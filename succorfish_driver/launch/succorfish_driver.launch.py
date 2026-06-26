"""Launch the succorfish_driver serial bridge.

Pick a hardware profile (``succorfish`` -> 9600/ttyUSB, ``teensy`` -> 115200/ttyACM)
which selects the bundled config file, or pass ``config_file`` to use your own.
Any of ``port``/``port_fallback``/``baudrate`` given on the CLI override the file.

Examples:
    ros2 launch succorfish_driver succorfish_driver.launch.py
    ros2 launch succorfish_driver succorfish_driver.launch.py profile:=teensy
    ros2 launch succorfish_driver succorfish_driver.launch.py \\
        profile:=succorfish port:=/dev/ttyUSB2 baudrate:=9600 namespace:=sam
    ros2 launch succorfish_driver succorfish_driver.launch.py \\
        config_file:=/abs/path/to/my_serial.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_PROFILE_FILES = {
    "succorfish": "succorfish_modem.yaml",
    "teensy": "teensy.yaml",
}


def _setup(context, *args, **kwargs):
    profile = LaunchConfiguration("profile").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    port = LaunchConfiguration("port").perform(context)
    port_fallback = LaunchConfiguration("port_fallback").perform(context)
    baudrate = LaunchConfiguration("baudrate").perform(context)

    if config_file:
        params_file = config_file
    else:
        fname = _PROFILE_FILES.get(profile)
        if fname is None:
            raise RuntimeError(
                f"Unknown profile '{profile}'. Use one of "
                f"{sorted(_PROFILE_FILES)} or pass config_file:=...")
        params_file = os.path.join(
            get_package_share_directory("succorfish_driver"), "config", fname)

    overrides = {}
    if port:
        overrides["serial.port"] = port
    if port_fallback:
        overrides["serial.port_fallback"] = port_fallback
    if baudrate:
        overrides["serial.baudrate"] = int(baudrate)

    parameters = [params_file]
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package="succorfish_driver",
            executable="succorfish_driver_node",
            name="succorfish_driver_node",
            namespace=namespace,
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "profile", default_value="succorfish",
            description="Hardware profile: 'succorfish' (9600/ttyUSB) or 'teensy' (115200/ttyACM)."),
        DeclareLaunchArgument(
            "config_file", default_value="",
            description="Absolute path to a params YAML; overrides 'profile' when set."),
        DeclareLaunchArgument(
            "namespace", default_value="",
            description="ROS namespace for the driver node (e.g. the robot name)."),
        DeclareLaunchArgument(
            "port", default_value="",
            description="Override serial.port from the config file."),
        DeclareLaunchArgument(
            "port_fallback", default_value="",
            description="Override serial.port_fallback from the config file."),
        DeclareLaunchArgument(
            "baudrate", default_value="",
            description="Override serial.baudrate from the config file."),
        OpaqueFunction(function=_setup),
    ])
