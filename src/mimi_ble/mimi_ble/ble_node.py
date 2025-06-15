import rclpy
from rclpy.node import Node
from mimi_interfaces.msg import Drive
from mimi_ble.ble_client import BleClient
import asyncio


class BleNode(Node):

    def __init__(self):
        super().__init__('ble_node')
        self.subscription = self.create_subscription(Drive, '/drive', self.listener_callback, 10)
        self.loop = asyncio.get_event_loop()
        self.ble = BleClient(robot_name="BBC micro:bit")
        self.ble.set_rx_callback(self.on_ble_message)
        self.loop.create_task(self.ble.start())

    def listener_callback(self, msg: Drive):
        self.get_logger().info('Drive: [%.1f, %.1f]' % (msg.left_speed, msg.right_speed))
        # self.loop.create_task(self.ble.send(f"drive {msg.left_speed:.2f} {msg.right_speed:.2f}"))

    def on_ble_message(self, message: str):
        self.get_logger().info(f"From robot: {message}")


def main(args=None):
    rclpy.init(args=args)
    node = BleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
