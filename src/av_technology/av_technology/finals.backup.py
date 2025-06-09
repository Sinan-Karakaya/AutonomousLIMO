import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from enum import Enum
from tf_transformations import euler_from_quaternion
import math

TURN_DISTANCE_TRESHOLD = 0.55

class TurnDirections(Enum):
    NONE = 0
    LEFT = 1
    RIGHT = 2

class FinalExams(Node):
    def __init__(self):
        self.twist_msg = Twist()
        self.distance_to_obstacle = 0.0
        self.traffic_light_state = "NONE"
        self.already_stopped_at_traffic = False
        
        # Variables for turn handling
        self.turns_step = 0
        self.turns_map = [
            TurnDirections.RIGHT, TurnDirections.LEFT,
            TurnDirections.RIGHT, TurnDirections.RIGHT, 
            TurnDirections.RIGHT
        ]
        self.initial_yaw = None
        self.target_yaw = None
        self.turning = False
        self.angle_tolerance = 0.001
        
        # Subscribe camera data
        super().__init__('finals')
        self.subscription = self.create_subscription(
            Float32,
            'distance_to_obstacle',
            lambda msg: setattr(self, 'distance_to_obstacle', msg.data),
            rclpy.qos.qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            String,
            'traffic_light_state',
            lambda msg: setattr(self, 'traffic_light_state', msg.data),
            rclpy.qos.qos_profile_sensor_data)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscription # to prevent from warning

        self.timer_ = self.create_timer(0.01, self.timer_callback)
    
    def timer_callback(self):
        # self.get_logger().info(f"isTurning: {self.turning}, distance: {self.distance_to_obstacle:.2f} m, step: {self.turns_step}")
        if self.distance_to_obstacle <= TURN_DISTANCE_TRESHOLD and self.distance_to_obstacle > 0.0:
            self.turning = True

        # self.get_logger().info(f"Traffic light state: {self.traffic_light_state}")
        if self.already_stopped_at_traffic == False:
            if self.traffic_light_state == "RED":
                self.get_logger().info("Traffic light is RED, stopping.")
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = 0.0
                self.turning = False
                self.already_stopped_at_traffic = True

            if self.traffic_light_state == "GREEN":
                self.already_stopped_at_traffic = True
                self.turning = True 

        # self.twist_publisher.publish(self.twist_msg)
        self.twist_msg.linear.x = 0.1
        if self.turning == False:
            self.twist_msg.linear.x = 0.35
            self.twist_msg.angular.z = 0.0
        self.twist_publisher.publish(self.twist_msg)

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

        if self.turning:
            if self.initial_yaw is None:
                self.initial_yaw = yaw
                direction = self.turns_map[self.turns_step]
                angle_offset = math.radians(82) if direction == TurnDirections.LEFT else -math.radians(82)
                self.target_yaw = self.normalize_angle(self.initial_yaw + angle_offset)
                self.get_logger().info(f"Turning to target yaw: {math.degrees(self.target_yaw):.2f}°")

            current_yaw = self.normalize_angle(yaw)
            error = self.shortest_angular_distance(current_yaw, self.target_yaw)

            self.get_logger().info(f"Current yaw: {math.degrees(current_yaw):.2f}°, Target yaw: {math.degrees(self.target_yaw):.2f}°, Error: {math.degrees(error):.2f}°")
            if abs(error) > self.angle_tolerance:
                self.twist_msg.angular.z = 0.4 if error > 0 else -0.4
                # self.twist_publisher.publish(self.twist_msg)
            else:
                self.twist_msg.angular.z = 0.0
                self.turning = False
                self.initial_yaw = None
                self.target_yaw = None
                self.turns_step = self.turns_step + 1
                self.get_logger().info("Finished 90° turn.")

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def shortest_angular_distance(from_angle, to_angle):
        """Returns shortest distance (positive or negative) between two angles"""
        diff = to_angle - from_angle
        return math.atan2(math.sin(diff), math.cos(diff))


def main(args=None):
    rclpy.init(args=args)

    autonomous = FinalExams()
    rclpy.spin(autonomous)
    
    autonomous.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()