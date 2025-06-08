from launch import LaunchDescription  # type: ignore
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    joy_node = Node(
        package="joy",
        executable="joy_node",
    )
    ld.add_action(joy_node)

    gamepad_node = Node(
        package="mimi_common",
        executable="gamepad_node",
    )
    ld.add_action(gamepad_node)

    ble_node = Node(
        package="mimi_ble",
        executable="ble_node",
    )
    ld.add_action(ble_node)

    return ld
