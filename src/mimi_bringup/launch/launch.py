from launch import LaunchDescription  # type: ignore
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("mimi_bringup"),
        "config",
        "params.yaml"
    )

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
        # name="ble_node",
        parameters=[config],
    )
    ld.add_action(ble_node)

    return ld
