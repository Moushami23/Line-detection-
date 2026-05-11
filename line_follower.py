import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        self.bridge = CvBridge()

        # Subscribe to camera
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)

        # Publish movement
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Line follower started")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        height, width, _ = frame.shape

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Threshold (black line detection)
        _, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)

        # Region of interest (bottom half)
        roi = thresh[int(height/2):height, :]

        # Find contours
        contours, _ = cv2.findContours(roi, 1, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)

            M = cv2.moments(c)

            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])

                error = cx - width / 2

                twist.linear.x = 0.2
                twist.angular.z = -error / 100

        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
