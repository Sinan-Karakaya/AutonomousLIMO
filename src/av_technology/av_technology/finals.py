import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class FinalsNode(Node):
    def __init__(self):
        super().__init__('finals_node')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.traffic_light_state = "NONE"
        self.qrcode_data = 0

        # Those are for bonus points
        self.bridge = CvBridge()
        self.state_publisher = self.create_publisher(Image, '/finals_state', 10)
        self.color_state = (255, 0, 0)
        self.timer = self.create_timer(1.0, self.publish_image)

        self.subscription = self.create_subscription(
            String,
            '/traffic_light_state',
            self.traffic_light_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Int32,
            '/qr_code_detected',
            self.qrcode_callback,
            rclpy.qos.qos_profile_sensor_data)

    def send_single_waypoint(self, pose, bt=""):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = [pose]
        goal_msg.behavior_tree = bt
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
    
    def traffic_light_callback(self, msg):
        self.traffic_light_state = msg.data

    def qrcode_callback(self, msg):
        self.qrcode_data = msg.data

    def publish_image(self):
        from PIL import Image as PILImage
        import numpy as np
        img = PILImage.new('RGB', (1080, 480), self.color_state)
        img_bgr = np.array(img)[..., ::-1]
        img_msg = self.bridge.cv2_to_imgmsg(np.array(img_bgr), encoding='bgr8')
        self.state_publisher.publish(img_msg)

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
    navigator = FinalsNode()

    default_waypoints = [
        create_pose(1.49, 0.28, -1.57), # First turn
        create_pose(1.49, -0.53, -1.57), # Traffic light check stop
        create_pose(1.733, -1.0, 0), # Front of traffic light
        create_pose(2.82, -0.83, -1.57), # Green brick after traffic light
        create_pose(2.97, -2.24, -1.57), # Mid point top side
        create_pose(3.13, -3.23, -1.57*2), # Between 2 yellow bricks
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
        create_pose(1.08, -1.13, 1.57), # Front of spot
        create_pose(0.96, -0.54, 1.57), # Parking spot
        create_pose(0.96, -0.54, -1.57), # Parking
    ]

    i = 0
    for pose in default_waypoints:
        navigator.send_single_waypoint(pose)
        if i == 1:
            start = time.time()
            while navigator.traffic_light_state != "GREEN":
                navigator.get_logger().info(f"Traffic light status: {navigator.traffic_light_state}")
                rclpy.spin_once(navigator)
                if time.time() - start > 15:
                    navigator.traffic_light_state = "GREEN"  # Force green if stuck for more than 15 seconds
        i = i + 1
    
    while navigator.qrcode_data != 1 and navigator.qrcode_data != 2:
        navigator.get_logger().info(f"Waiting for QR code data... Current data: {navigator.qrcode_data}")
        rclpy.spin_once(navigator)

    i = 0
    navigator.get_logger().info(f"QR Code detected: {navigator.qrcode_data}")
    current_path = first_parking_waypoints if navigator.qrcode_data == 1 else second_parking_waypoints
    for pose in current_path:
        if i == 2:
            navigator.get_logger().info("Parking completed, turning around")
            navigator.send_single_waypoint(pose, "precise_bt.xml")
        else:
            navigator.send_single_waypoint(pose)
        i = i + 1

    navigator.color_state = (0, 255, 0)
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()