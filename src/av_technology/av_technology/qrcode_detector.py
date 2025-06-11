import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
from std_msgs.msg import String
import cv2

class QRCodeDetector(Node):
    def __init__(self):
        super().__init__('qr_code_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.qrcode_publisher = self.create_publisher(String, '/qr_code_detected', 10)

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        barcodes = decode(cv_image)

        for barcode in barcodes:
            qr_data = barcode.data.decode('utf-8')
            if qr_data is not None:
                qr_code_msg = String()
                qr_code_msg.data = qr_data
                self.get_logger().info(f'Detected QR code: {qr_data}')
                self.qrcode_publisher.publish(qr_code_msg)
            else:
                continue
            

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()