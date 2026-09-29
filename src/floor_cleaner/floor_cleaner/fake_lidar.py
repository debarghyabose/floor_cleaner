import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class FakeLidar(Node):

    def __init__(self):

        super().__init__('fake_lidar')

        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_scan
        )

        self.get_logger().info('Fake LiDAR Started')

    def publish_scan(self):

        scan = LaserScan()

        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'

        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(1)

        scan.range_min = 0.05
        scan.range_max = 10.0

        number_of_readings = 360

        scan.ranges = [5.0] * number_of_readings

        self.publisher.publish(scan)


def main(args=None):

    rclpy.init(args=args)

    node = FakeLidar()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
