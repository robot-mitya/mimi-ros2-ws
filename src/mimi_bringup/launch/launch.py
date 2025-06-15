from launch import LaunchDescription  # type: ignore
from launch_ros.actions import Node
import os

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

    # Prepare environment for mimi_ble (venv + PYTHONPATH)
    ws_root = os.path.dirname(os.path.dirname(__file__))
    venv_path = os.path.join(ws_root, ".venv")
    python_site = os.path.join(venv_path, "lib", "python3.12", "site-packages")

    custom_env = os.environ.copy()
    custom_env["PATH"] = os.path.join(venv_path, "bin") + ":" + custom_env["PATH"]
    custom_env["PYTHONPATH"] = python_site + ":" + custom_env.get("PYTHONPATH", "")

    ble_node = Node(
        package="mimi_ble",
        executable="ble_node",
        env=custom_env,
        output="screen",
    )
    ld.add_action(ble_node)

    return ld
