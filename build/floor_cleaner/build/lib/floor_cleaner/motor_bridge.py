import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotorBridge(Node):

    def __init__(self):

        super().__init__('motor_bridge')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('Motor Bridge Started')

    def cmd_vel_callback(self, msg):

        linear = msg.linear.x
        angular = msg.angular.z

        self.get_logger().info(
            f'linear={linear:.2f}, angular={angular:.2f}'
        )

        if linear > 0.05:
            self.move_forward()

        elif linear < -0.05:
            self.move_backward()

        elif angular > 0.05:
            self.turn_left()

        elif angular < -0.05:
            self.turn_right()

        else:
            self.stop()

    def move_forward(self):
        self.get_logger().info('FORWARD')

    def move_backward(self):
        self.get_logger().info('BACKWARD')

    def turn_left(self):
        self.get_logger().info('LEFT')

    def turn_right(self):
        self.get_logger().info('RIGHT')

    def stop(self):
        self.get_logger().info('STOP')


def main(args=None):

    rclpy.init(args=args)

    node = MotorBridge()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
