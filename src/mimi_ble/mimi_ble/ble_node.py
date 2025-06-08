import rclpy
from rclpy.node import Node

from mimi_interfaces.msg import Drive


class BleNode(Node):

    def __init__(self):
        super().__init__('ble_node')
        self.subscription = self.create_subscription(
            Drive,
            '/drive',
            self.listener_callback,
            10)
        # noinspection PyStatementEffect
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('Drive: [%4.1f, %4.1f]' % (msg.left_speed, msg.right_speed))


def main(args=None):
    rclpy.init(args=args)

    ble_node = BleNode()

    rclpy.spin(ble_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ble_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
