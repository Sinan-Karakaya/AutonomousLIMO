import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(String, '/traffic_light_state', 10)
        self.debug_pub = self.create_publisher(Image, '/debug_traffic_light', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Apply ROI: only take the top 100 pixels
        roi = frame[:30, :]
        debug_img = roi.copy()
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Red light range (two intervals needed in HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        red_mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

        # Yellow light range
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Green arrow range
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([90, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        # Draw rectangles around detected regions
        def draw_bounding_rects(mask, color):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                if cv2.contourArea(cnt) > 50:  # filter out small noise
                    cv2.rectangle(debug_img, (x, y), (x + w, y + h), color, 2)

        draw_bounding_rects(red_mask, (0, 0, 255))      # Red rectangles
        draw_bounding_rects(yellow_mask, (0, 255, 255)) # Yellow rectangles
        draw_bounding_rects(green_mask, (0, 255, 0))    # Green rectangles

        # Threshold: if enough pixels are detected, treat it as active
        self.get_logger().info(f"Red pixels: {cv2.countNonZero(red_mask)}, Yellow pixels: {cv2.countNonZero(yellow_mask)}, Green pixels: {cv2.countNonZero(green_mask)}")
        if cv2.countNonZero(red_mask) > 300 and cv2.countNonZero(red_mask) > cv2.countNonZero(green_mask):
            state = "RED"
        elif cv2.countNonZero(yellow_mask) > 300:
            state = "YELLOW"
        elif cv2.countNonZero(green_mask) > 300 and cv2.countNonZero(green_mask) > cv2.countNonZero(red_mask):
            state = "GREEN"
        else:
            state = "NONE"

        # Publish the detected state
        self.publisher.publish(String(data=state))

        # Publish the debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        self.debug_pub.publish(debug_msg)

    def detect_light_state(self, roi):
        """Detect state based on red circle (left) and green arrow (right) in a horizontal traffic light."""
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h, w, _ = roi.shape

        # Split into thirds: left (red), center (ignored), right (green arrow)
        left = hsv_roi[:, :w // 3]
        right = hsv_roi[:, 2 * w // 3:]

        # Red detection in the left region
        red_mask = cv2.inRange(left, np.array((0, 100, 100)), np.array((10, 255, 255)))
        red_count = cv2.countNonZero(red_mask)

        # Green detection in the right region
        green_mask = cv2.inRange(right, np.array((50, 100, 100)), np.array((70, 255, 255)))
        green_count = cv2.countNonZero(green_mask)

        # Try detecting arrow shape in the green region (optional shape check)
        green_arrow = False
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.05 * cv2.arcLength(cnt, True), True)
            if 5 <= len(approx) <= 8:  # approximate arrow shape
                green_arrow = True
                break

        if red_count > 500:
            return "red"
        elif green_count > 500 and green_arrow:
            return "green_left_arrow"
        else:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()