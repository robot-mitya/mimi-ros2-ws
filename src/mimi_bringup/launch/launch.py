from launch import LaunchDescription  # type: ignore
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    joy_node = Node(
        package="joy",
        executable="joy_node",
    )
    ld.add_action(joy_node)

    listener_node = Node(
        package="mimi_common",
        executable="gamepad_node",
    )
    ld.add_action(listener_node)

    return ld
