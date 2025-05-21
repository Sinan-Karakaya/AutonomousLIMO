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
        debug_img = frame.copy()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV ranges
        color_ranges = {
            "red":    [(0, 100, 100), (10, 255, 255)],
            "yellow": [(20, 100, 100), (30, 255, 255)],
            "green":  [(50, 100, 100), (70, 255, 255)],
        }

        masks = {}
        for color, (lower, upper) in color_ranges.items():
            masks[color] = cv2.inRange(hsv, np.array(lower), np.array(upper))

        combined_mask = cv2.bitwise_or(masks["red"], cv2.bitwise_or(masks["yellow"], masks["green"]))
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = w / float(h)

            if aspect_ratio > 2.5 and w > 30 and h > 10:
                cv2.rectangle(debug_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                roi = frame[y:y+h, x:x+w]
                state = self.detect_light_state(roi)

                if state:
                    cv2.putText(debug_img, state, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 255, 0), 2)
                    self.publisher.publish(String(data=state))
                    self.get_logger().info(f"Detected traffic light: {state}")
                    break

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