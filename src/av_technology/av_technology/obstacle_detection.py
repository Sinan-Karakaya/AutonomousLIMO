import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class ObstacleDetection(Node):
    def __init__(self):
        # Required variables for laser scan
        self.safe_distance = 0.55    # Minimum gap width
        self.angle_range = 25  # Degrees to consider around fronts
        self.distance_to_obstacle = 0.0
        
        # Subscribe camera data
        super().__init__('detect_line')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.subscription # to prevent from warning

        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.distance_to_obstacle_publisher = self.create_publisher(Float32, 'distance_to_obstacle', 10)
    
    def timer_callback(self):
        msg = Float32()
        msg.data = self.distance_to_obstacle
        self.distance_to_obstacle_publisher.publish(msg)

    def laser_callback(self, msg):
        ranges = np.array(msg.ranges)

        # Replace NaN or inf with max range
        ranges = np.nan_to_num(ranges, nan=msg.range_max, posinf=msg.range_max)

        # Get the center beam (directly in front)
        center_index = len(ranges) // 2
        center_distance = ranges[center_index]

        # Apply a threshold for too-close detection
        if center_distance <= 0.1:
            self.distance_to_obstacle = -1.0  # Treat as collision or invalid
        else:
            self.distance_to_obstacle = float(center_distance)

def main(args=None):
    rclpy.init(args=args)

    autonomous = ObstacleDetection()
    rclpy.spin(autonomous)
    
    autonomous.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()