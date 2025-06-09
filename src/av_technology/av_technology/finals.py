import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

    def send_single_waypoint(self, pose):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = [pose]
        goal_msg.behavior_tree = ''
        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        self.get_logger().info(f'Result: {result}')

import math
def quaternion_to_yaw(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

def create_pose(x, y, yaw):
    from tf_transformations import quaternion_from_euler
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = Clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    q = quaternion_from_euler(0, 0, yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()

    # Example waypoints (x, y, yaw in radians)
    waypoints = [
        create_pose(1.42, -0.250, -1.496),
        create_pose(1.62, -1.0, 0.129),
        # create_pose(1.0, 1.0, 1.57),
        # create_pose(0.0, 1.0, 3.14),
    ]

    for pose in waypoints:
        navigator.send_single_waypoint(pose)

    rclpy.shutdown()

if __name__ == '__main__':
    main()