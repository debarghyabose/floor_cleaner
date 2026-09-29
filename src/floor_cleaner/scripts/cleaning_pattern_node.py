#!/usr/bin/env python3
"""
cleaning_pattern_node.py
========================
A deliberately simple "zig-zag" (boustrophedon) coverage node.

It does NOT drive the motors itself and does NOT plan paths around obstacles.
It only decides WHERE the robot should go, then asks Nav2 to go there:

    cleaning node  --NavigateToPose goal-->  Nav2 (bt_navigator)  --/cmd_vel-->  robot

Pattern inside the rectangle [x_min, x_max] x [y_min, y_max] (map frame):

    lane 0:  (x_min, y0) -> (x_max, y0)        → → → → →
                                                         ↓
    lane 1:  (x_max, y1) -> (x_min, y1)        ← ← ← ← ←
                                               ↓
    lane 2:  (x_min, y2) -> (x_max, y2)        → → → → →
    ...

Every lane end is one Nav2 goal. Goals are sent one after another; if a goal
fails (e.g. it is inside furniture) the node logs it and continues with the next.

The planned zig-zag is also published on /cleaning_path (nav_msgs/Path) so you
can see it in RViz2 before and while the robot drives.

Run (with Nav2 running and the robot localised):
    ros2 launch floor_cleaner cleaning.launch.py
or
    ros2 run floor_cleaner cleaning_pattern_node.py --ros-args -p use_sim_time:=true
"""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def generate_zigzag(x_min, x_max, y_min, y_max, lane_spacing, start_left=True):
    """Return a list of (x, y, yaw) waypoints covering the rectangle lane by lane.

    Pure Python (no ROS), so it is easy to test and to reuse later.
    """
    if x_max <= x_min or y_max < y_min:
        raise ValueError('cleaning area is empty: check x_min/x_max/y_min/y_max')
    if lane_spacing <= 0.0:
        raise ValueError('lane_spacing must be > 0')

    # lane y-coordinates from y_min to y_max (always include y_max)
    lanes = []
    y = y_min
    while y < y_max - 1e-6:
        lanes.append(y)
        y += lane_spacing
    lanes.append(y_max)

    waypoints = []
    left_to_right = start_left
    for lane_y in lanes:
        if left_to_right:
            start_x, end_x, heading = x_min, x_max, 0.0
        else:
            start_x, end_x, heading = x_max, x_min, math.pi
        waypoints.append((start_x, lane_y, heading))
        waypoints.append((end_x, lane_y, heading))
        left_to_right = not left_to_right
    return waypoints


def yaw_to_quaternion(yaw):
    """Rotation about Z only -> (x, y, z, w) quaternion."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class CleaningPatternNode(Node):

    def __init__(self):
        super().__init__('cleaning_pattern_node')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('x_min', -4.3)
        self.declare_parameter('x_max', 4.3)
        self.declare_parameter('y_min', -0.9)
        self.declare_parameter('y_max', 0.9)
        self.declare_parameter('lane_spacing', 0.45)
        self.declare_parameter('start_left', True)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('goal_timeout_sec', 120.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.dry_run = self.get_parameter('dry_run').value
        self.goal_timeout = self.get_parameter('goal_timeout_sec').value

        self.waypoints = generate_zigzag(
            float(self.get_parameter('x_min').value),
            float(self.get_parameter('x_max').value),
            float(self.get_parameter('y_min').value),
            float(self.get_parameter('y_max').value),
            float(self.get_parameter('lane_spacing').value),
            bool(self.get_parameter('start_left').value),
        )

        # latched (transient local) so RViz2 shows it even if opened later
        path_qos = QoSProfile(depth=1,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.path_pub = self.create_publisher(Path, 'cleaning_path', path_qos)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._last_feedback_log = self.get_clock().now()

    # ------------------------------------------------------------------
    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def publish_path(self):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [self.make_pose(x, y, yaw) for x, y, yaw in self.waypoints]
        self.path_pub.publish(path)

    def feedback_callback(self, feedback_msg):
        now = self.get_clock().now()
        if (now - self._last_feedback_log) > Duration(seconds=2.0):
            self._last_feedback_log = now
            self.get_logger().info(
                f'    distance remaining: {feedback_msg.feedback.distance_remaining:.2f} m')

    # ------------------------------------------------------------------
    def go_to(self, x, y, yaw):
        """Send one NavigateToPose goal and block until it finishes.

        Returns True on success, False otherwise.
        """
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(x, y, yaw)

        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('    goal was rejected by Nav2')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future,
                                         timeout_sec=self.goal_timeout)
        if not result_future.done():
            self.get_logger().warn('    goal timed out, cancelling it')
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            return False

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return True
        self.get_logger().warn(f'    goal finished with status {status} (not succeeded)')
        return False

    def run(self):
        self.publish_path()
        self.get_logger().info(
            f'Zig-zag plan: {len(self.waypoints)} waypoints in frame "{self.frame_id}"')
        for i, (x, y, yaw) in enumerate(self.waypoints):
            self.get_logger().info(f'  [{i}] x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f} deg')

        if self.dry_run:
            self.get_logger().info('dry_run is true: path published, robot will not move.')
            return

        self.get_logger().info('Waiting for the Nav2 action server "navigate_to_pose"...')
        while rclpy.ok() and not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('  ...still waiting (is navigation.launch.py running?)')

        succeeded = 0
        for i, (x, y, yaw) in enumerate(self.waypoints):
            if not rclpy.ok():
                break
            self.get_logger().info(
                f'Waypoint {i + 1}/{len(self.waypoints)}: going to ({x:.2f}, {y:.2f})')
            if self.go_to(x, y, yaw):
                succeeded += 1
                self.get_logger().info('    reached')
            else:
                self.get_logger().warn('    skipped, continuing with the next waypoint')

        self.get_logger().info(
            f'Cleaning pattern finished: {succeeded}/{len(self.waypoints)} waypoints reached.')


def main(args=None):
    rclpy.init(args=args)
    node = CleaningPatternNode()
    try:
        node.run()
        if node.dry_run:
            # keep the node alive so the latched path stays visible in RViz2
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
