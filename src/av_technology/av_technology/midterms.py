import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32
from enum import Enum

SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480

def smooth_obstacle_mask(safe, min_obstacle_width=5):
    # Invert safe → obstacle mask (1 = obstacle, 0 = safe)
    obstacle_mask = (~safe).astype(np.uint8)

    # Apply morphological opening to remove small obstacles
    kernel = np.ones((1, min_obstacle_width), np.uint8)
    cleaned_obstacle_mask = cv2.morphologyEx(obstacle_mask.reshape(1, -1), cv2.MORPH_OPEN, kernel)

    # Invert back to safe region
    cleaned_safe = ~cleaned_obstacle_mask.flatten().astype(bool)
    return cleaned_safe

class LaneStatus(Enum):
    NONE = 0
    LOST_LEFT = 1
    LOST_RIGHT = 2
    LOST_BOTH = 3

class Autonomous(Node):
    def __init__(self):
        # Required variables for lane detection
        self.br = CvBridge()
        self.distance = 0
        self.steer = 0.0
        self.lane_status = LaneStatus.NONE
        self.last_lane_lost = LaneStatus.NONE

        # Required variables for laser scan
        self.safe_distance = 0.55    # Minimum gap width
        self.angle_range = 25  # Degrees to consider around fronts
        self.avoid_direction = LaneStatus.NONE  # We set this value based on which lane is the farthest from the center
        
        # Subscribe camera data
        super().__init__('detect_line')
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
                            Image,
                            '/camera/color/image_raw', 
                            self.image_callback, 
                            rclpy.qos.qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.subscription # to prevent from warning

        # Publish result (offset between reference distance and real distance)
        self.dis_publisher = self.create_publisher(Int32, 'distance_y', 10)
        self.dis_subscriber = self.create_subscription(Int32, 'distance_y', self.distance_callback, 10)

        # Publish steering angle to avoid obstacles
        self.steer_publisher = self.create_publisher(Float32, 'steer_avoid', 10)
        self.steer_subscriber = self.create_subscription(Float32, 'steer_avoid', self.steer_callback, 10)

        # Publish Image for debugging
        self.debug_publisher = self.create_publisher(Image, 'debug_image', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        # Parameters (For Masking Lane, For the reference distance of lane, ROI)
        self.roi_left = np.array([0, SCREEN_WIDTH // 2, 400, 480])
        self.roi_right = np.array([SCREEN_WIDTH // 2, SCREEN_WIDTH, 400, 480])

        self.lane_hls_low = np.array([20, 40, 60])
        self.lane_hls_high = np.array([65, 255, 220])
        self.yellow_lane_low = np.array([self.lane_hls_low[0],
                                         self.lane_hls_low[1],
                                         self.lane_hls_low[2]])
        self.yellow_lane_high = np.array([self.lane_hls_high[0],
                                          self.lane_hls_high[1],
                                          self.lane_hls_high[2]])
        
        self.reference_distance = SCREEN_WIDTH // 2

        self.declare_parameter('debug_image_num', 2) #default 2
        self.debug_sequence = self.get_parameter('debug_image_num')
        self.sub_flag = False
    
    def timer_callback(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.36   # base speed
        twist_msg.angular.z = (self.distance / 130.0)

        if self.avoid_direction == LaneStatus.LOST_LEFT:
            twist_msg.angular.z += self.steer * 1.7
        elif self.avoid_direction == LaneStatus.LOST_RIGHT:
            twist_msg.angular.z += -self.steer * 1.7

        self.vel_publisher.publish(twist_msg)

        if self.sub_flag:     
                self.debug_publisher.publish(self.br.cv2_to_imgmsg(self.image_,'bgr8'))
          
    def symbol_callback(self, msg):
        self.get_logger().info(f"Test: {msg.data}")

    def image_callback(self, msg):
        if self.lane_status != LaneStatus.NONE and self.lane_status != LaneStatus.LOST_BOTH:
            self.last_lane_lost = self.lane_status
        self.lane_status = LaneStatus.NONE
        distance_to_ref = 0
        self.image_ = self.br.imgmsg_to_cv2(msg, 'bgr8')    # Convert ROS image to OpenCV format

        # Define and extract left ROI
        roi_left_img = self.image_[self.roi_left[2]:self.roi_left[3],
                                self.roi_left[0]:self.roi_left[1]]
        # Define and extract right ROI
        roi_right_img = self.image_[self.roi_right[2]:self.roi_right[3],
                                    self.roi_right[0]:self.roi_right[1]]

        # Convert to HLS and apply yellow mask
        hls_left = cv2.cvtColor(roi_left_img, cv2.COLOR_BGR2HLS)
        hls_right = cv2.cvtColor(roi_right_img, cv2.COLOR_BGR2HLS)
        mask_left = cv2.inRange(hls_left, self.yellow_lane_low, self.yellow_lane_high)
        mask_right = cv2.inRange(hls_right, self.yellow_lane_low, self.yellow_lane_high)
        center_x, center_y = 320, 440

        # Morphological filtering
        kernel = np.ones((5, 5), np.uint8)
        mask_clean_left = cv2.morphologyEx(mask_left, cv2.MORPH_OPEN, kernel)
        mask_left = cv2.morphologyEx(mask_clean_left, cv2.MORPH_CLOSE, kernel)
        mask_clean_right = cv2.morphologyEx(mask_right, cv2.MORPH_OPEN, kernel)
        mask_right = cv2.morphologyEx(mask_clean_right, cv2.MORPH_CLOSE, kernel)

        # Edge + Color combination
        left_edges = cv2.Canny(roi_left_img, 50, 150)
        mask_left = cv2.bitwise_and(left_edges, mask_left)
        right_edges = cv2.Canny(roi_right_img, 50, 150)
        mask_right = cv2.bitwise_and(right_edges, mask_right)

        # Process left lane
        M_left = cv2.moments(mask_left)
        cx_left, cy_left = None, None
        if M_left['m00'] > 0:
            cx_left = int(M_left['m10'] / M_left['m00']) + self.roi_left[0]
            cy_left = int(M_left['m01'] / M_left['m00']) + self.roi_left[2]

        # Process right lane
        M_right = cv2.moments(mask_right)
        cx_right, cy_right = None, None
        if M_right['m00'] > 0:
            cx_right = int(M_right['m10'] / M_right['m00']) + self.roi_right[0]
            cy_right = int(M_right['m01'] / M_right['m00']) + self.roi_right[2]

        if cx_left is None and cx_right is None:
            if self.last_lane_lost == LaneStatus.LOST_LEFT:
                self.lane_status = LaneStatus.LOST_RIGHT
            elif self.last_lane_lost == LaneStatus.LOST_RIGHT:
                self.lane_status = LaneStatus.LOST_LEFT

        # Check if the 2 lanes detected are the same one
        # 1. Convert both ROIs to HLS
        hls_left = cv2.cvtColor(roi_left_img, cv2.COLOR_BGR2HLS)
        hls_right = cv2.cvtColor(roi_right_img, cv2.COLOR_BGR2HLS)

        # 2. Apply yellow lane mask to both
        mask_left = cv2.inRange(hls_left, self.yellow_lane_low, self.yellow_lane_high)
        mask_right = cv2.inRange(hls_right, self.yellow_lane_low, self.yellow_lane_high)

        # 3. Extract the relevant edge columns (binary 0 or 255)
        edge_left = mask_left[:, -1]   # Rightmost column of left ROI
        edge_right = mask_right[:, 0] # Leftmost column of right ROI

        # 4. Normalize binary values to 0 and 1
        edge_left = edge_left // 255
        edge_right = edge_right // 255

        # 5. Compute similarity (MAE or simple overlap)
        mae = np.mean(np.abs(edge_left - edge_right))
        only_one_lane = mae > 0.01
        if mae > 0.01:  # Threshold for MAE
            center_y = 480
            if self.last_lane_lost == LaneStatus.LOST_LEFT:
                center_x = cx_right - 600 if cx_right is not None else -800
            elif self.last_lane_lost == LaneStatus.LOST_RIGHT:
                center_x = cx_left + 600 if cx_left is not None else 800
            elif self.last_lane_lost == LaneStatus.LOST_BOTH:
                # if self.last_lane_lost == LaneStatus.LOST_LEFT:
                #     center_x = 800
                # elif self.last_lane_lost == LaneStatus.LOST_RIGHT:
                #     center_x = -800
                pass

        # Check for fallbacks if we lost a lane, and if the only lane doesn't cross our ROIs
        if only_one_lane == False:
            if cx_left is None and cx_right is not None:
                cx_left = cx_right - SCREEN_WIDTH // 2 - SCREEN_WIDTH // 8
                cy_left = cy_right
                center_x = (cx_left + cx_right) // 2
                center_y = cy_left
                self.lane_status = LaneStatus.LOST_LEFT
            elif cx_right is None and cx_left is not None:
                cx_right = cx_left + SCREEN_WIDTH // 2 + SCREEN_WIDTH // 8
                cy_right = cy_left
                center_x = (cx_left + cx_right) // 2
                center_y = cy_right
                self.lane_status = LaneStatus.LOST_RIGHT
            elif cx_right is None and cx_left is None:
                if self.last_lane_lost == LaneStatus.LOST_LEFT:
                    cx_left = 40
                    cy_left = SCREEN_HEIGHT
                    cx_right = cx_left + SCREEN_WIDTH // 2 + SCREEN_WIDTH // 8
                    cy_right = cy_left
                    center_x = (cx_left + cx_right) // 2
                    center_y = cy_left
                    self.lane_status = LaneStatus.LOST_LEFT
                else:
                    cx_right = SCREEN_WIDTH - 40
                    cy_right = SCREEN_HEIGHT
                    cx_left = cx_right - SCREEN_WIDTH // 2 - SCREEN_WIDTH // 8
                    cy_left = cy_right
                    center_x = (cx_left + cx_right) // 2
                    center_y = cy_right
                    self.lane_status = LaneStatus.LOST_RIGHT
            elif cx_right is not None and cx_left is not None:
                # Compute center between lanes
                center_x = (cx_left + cx_right) // 2
                center_y = (cy_left + cy_right) // 2

        # Find which lane is the farthest from the center and set the avoid direction
        if cx_left is not None and cx_right is not None and only_one_lane == False:
            if self.last_lane_lost == LaneStatus.LOST_LEFT:
                self.avoid_direction = LaneStatus.LOST_LEFT
            else:
                self.avoid_direction = LaneStatus.LOST_RIGHT
        else:
            self.avoid_direction = LaneStatus.NONE

        cv2.circle(self.image_, (cx_right, cy_right), 7, (255, 0, 255), -1)  # Right centroid
        cv2.circle(self.image_, (cx_left, cy_left), 7, (255, 255, 0), -1)  # Left centroid
        cv2.circle(self.image_, (center_x, center_y), 10, (0, 255, 255), -1)  # Yellow center dot
        cv2.line(self.image_, (self.reference_distance, 0),
                (self.reference_distance, self.image_.shape[0]), (255, 0, 0), 2)
        distance_to_ref = self.reference_distance - center_x

        # Steering angle
        origin = (SCREEN_WIDTH // 2, SCREEN_HEIGHT)
        angle_rad = self.steer
        end_x = int(origin[0] + 100 * np.sin(angle_rad))
        end_y = int(origin[1] - 100 * np.cos(angle_rad))
        cv2.arrowedLine(self.image_, (SCREEN_WIDTH // 2, SCREEN_HEIGHT), 
                        (end_x, end_y), (0, 255, 0), 2, tipLength=0.1)

        # ROIs for visualization
        cv2.rectangle(self.image_,
                    (self.roi_left[0], self.roi_left[2]),
                    (self.roi_left[1], self.roi_left[3]),
                    (0, 255, 0), 2)
        cv2.rectangle(self.image_,
                    (self.roi_right[0], self.roi_right[2]),
                    (self.roi_right[1], self.roi_right[3]),
                    (0, 255, 0), 2)

        # Publishing the offset between current distance with lane and reference distance
        dis = Int32()
        dis.data = int(distance_to_ref) if distance_to_ref else 0
        self.dis_publisher.publish(dis)
        self.sub_flag = True

    def distance_callback(self, msg):
        self.distance = msg.data

    def steer_callback(self, msg):
        self.steer = msg.data

    def laser_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Focus on front view
        center_fov_mask = np.abs(np.degrees(angles)) < self.angle_range
        filtered_ranges = ranges[center_fov_mask]
        filtered_angles = angles[center_fov_mask]

        # Smooth noisy readings
        filtered_ranges = np.nan_to_num(filtered_ranges, nan=msg.range_max, posinf=msg.range_max)

        # Mark all unsafe distances as 0
        safe = filtered_ranges > self.safe_distance
        safe_mask = safe.astype(np.uint8)
        safe_mask = cv2.morphologyEx(safe_mask.reshape(1, -1), cv2.MORPH_CLOSE, np.ones((1, 5), np.uint8)).flatten()
        safe = safe_mask.astype(bool)
        safe = smooth_obstacle_mask(safe, min_obstacle_width=5)
        gaps = self.find_largest_gaps(safe)

        if gaps is not None:
            start_idx, end_idx = gaps
            mid_idx = (start_idx + end_idx) // 2
            steer_angle = filtered_angles[mid_idx]

            center_distance = np.clip(ranges[len(ranges) // 2], 0.02, 1.0)  # 5cm to 1m
            scaling = np.exp((1.0 - center_distance) * 2.5)  # sharper response for close obstacles
            scaling = np.clip(scaling, 1.0, 5.0)
            steer_angle = steer_angle * scaling
            
            dis = Float32()
            dis.data = abs(steer_angle)
            self.steer_publisher.publish(dis)
        else:
            self.get_logger().info("No safe gaps found")

    def find_largest_gaps(self, safe, min_gap_width=5):
        max_len = 0
        max_start = max_end = None
        start = None

        for i, val in enumerate(safe):
            if val:
                if start is None:
                    start = i
            else:
                if start is not None:
                    length = i - start
                    if length > max_len and length >= min_gap_width:
                        max_len = length
                        max_start, max_end = start, i - 1
                    start = None

        # Check for final open gap
        if start is not None:
            length = len(safe) - start
            if length > max_len and length >= min_gap_width:
                max_start, max_end = start, len(safe) - 1

        if max_start is not None:
            return (max_start, max_end)
        else:
            return None

def main(args=None):
    rclpy.init(args=args)

    autonomous = Autonomous()
    rclpy.spin(autonomous)
    
    autonomous.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()