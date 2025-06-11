import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from std_msgs.msg import String

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.traffic_light_state = "NONE"
        self.qrcode_data = ""

        self.subscription = self.create_subscription(
            String,
            'traffic_light_state',
            lambda msg: setattr(self, 'traffic_light_state', msg.data),
            rclpy.qos.qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            String,
            'qr_code_detected',
            lambda msg: setattr(self, 'qrcode_data', msg.data),
            rclpy.qos.qos_profile_sensor_data)

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

    def get_traffic_light_state(self):
        return self.traffic_light_state

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

    default_waypoints = [
        create_pose(1.324, 0.43, -1.57), # First turn
        create_pose(1.4628, -0.37, -1.57), # Traffic light check stop
        create_pose(1.733, -1.0, 0), # Front of traffic light
        create_pose(2.8, -0.95, -1.57), # Green brick after traffic light
        create_pose(3.1, -2.24, -1.57), # Mid point top side
        create_pose(3.20, -3.23, -1.57*2), # Between 2 yellow bricks
        create_pose(2.07, -3.46, -1.57*2), # Mid point right side
        create_pose(0.98, -3.7, 1.57), # Before QR code
        create_pose(0.8, -3.19, 1.57), # Before QR code
    ]

    first_parking_waypoints = [
        create_pose(0.81, -2.74, 0), # Before turn for parking,
        create_pose(1.38, -2.55, 1.57), # Front of spot
        create_pose(1.29, -1.99, 1.57), # Parking spot
        create_pose(1.29, -1.99, -1.57), # Parking
    ]

    second_parking_waypoints = [
        create_pose(0.48, -1.3, 0), # Before turn for parking,
        create_pose(1.05, -1.11, 1.57), # Front of spot
        create_pose(0.93, -0.63, 1.57), # Parking spot
        create_pose(0.93, -0.63, -1.57), # Parking
    ]

    while True:
        navigator.get_logger().info(f"Traffic light status: {navigator.traffic_light_state}")

    i = 0
    # for pose in default_waypoints:
    #     navigator.send_single_waypoint(pose)
    #     if i == 1:
    #         while navigator.traffic_light_state != "GREEN":
    #             navigator.get_logger().info(f"Traffic light status: {navigator.traffic_light_state}")
    #             pass
    #     i = i + 1
    
    # while navigator.qrcode_data != "":
    #     navigator.get_logger().info(f"Waiting for QR code data... Current data: {navigator.qrcode_data}")
    #     if navigator.qrcode_data != "":
    #         break

    # TODO: add custom BT for parking
    i = 0
    # navigator.get_logger().info(f"QR Code detected: {navigator.qrcode_data}")
    # if navigator.qrcode_data.find("1") != -1:
    #     navigator.get_logger().info("QR Code 1 detected, proceeding to next waypoint.")
    # for pose in first_parking_waypoints:
    #     navigator.send_single_waypoint(pose)
    #     i = i + 1

    # elif navigator.qrcode_data.find("2") != -1:
    #     navigator.get_logger().info("QR Code 2 detected, proceeding to next waypoint.")
    #     for pose in second_parking_waypoints:
    #         navigator.send_single_waypoint(pose)
    #         i = i + 1

    rclpy.shutdown()

if __name__ == '__main__':
    main()