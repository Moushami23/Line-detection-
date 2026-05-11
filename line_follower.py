import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23
ENA = 18
ENB = 24

SPD_STRAIGHT = 30
SPD_TURN = 35
SPD_BACK = 25

SEARCH_IDLE = 'IDLE'
SEARCH_LEFT = 'SEARCH_LEFT'
SEARCH_RIGHT = 'SEARCH_RIGHT'
SEARCH_BACK = 'SEARCH_BACK'


class LineDetectorNode(Node):

    def __init__(self):
        super().__init__('line_detector')
        self.bridge = CvBridge()
        self.cap = None
        for index in range(5):
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    self.cap = cap
                    self.get_logger().info('Camera found at index ' + str(index))
                    break
                cap.release()
        if self.cap is None:
            self.get_logger().error('No camera found!')
            return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)
        self.pwm_a = GPIO.PWM(ENA, 1000)
        self.pwm_b = GPIO.PWM(ENB, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        self.dir_pub = self.create_publisher(String, '/line/direction', 10)
        self.img_pub = self.create_publisher(Image, '/line/image', 10)
        self.search_state = SEARCH_IDLE
        self.search_start = None
        self.last_direction = ''
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Line follower started')

    def set_motors(self, ld, rd, sl, sr):
        if ld == 'F':
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
        elif ld == 'B':
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
        else:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW)
        if rd == 'F':
            GPIO.output(IN3, GPIO.HIGH)
            GPIO.output(IN4, GPIO.LOW)
        elif rd == 'B':
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.HIGH)
        else:
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(sl)
        self.pwm_b.ChangeDutyCycle(sr)

    def stop_motors(self):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)

    def move(self, direction):
        if direction == 'NO LINE':
            self.stop_motors()
            return
        if direction == 'STRAIGHT':
            self.set_motors('F', 'F', SPD_STRAIGHT, SPD_STRAIGHT)
        elif direction == 'LEFT':
            self.set_motors('S', 'F', 0, SPD_TURN)
        elif direction == 'RIGHT':
            self.set_motors('F', 'S', SPD_TURN, 0)
        elif direction == 'BACK':
            self.set_motors('B', 'B', SPD_BACK, SPD_BACK)
        else:
            self.stop_motors()

    def do_search(self):
        now = time.time()
        if self.search_state == SEARCH_IDLE:
            print('LINE NOT FOUND - motors STOPPED')
            self.stop_motors()
            self.search_state = SEARCH_LEFT
            self.search_start = now
        elif self.search_state == SEARCH_LEFT:
            self.stop_motors()
            if now - self.search_start > 1.2:
                print('Checked LEFT... no line')
                self.search_state = SEARCH_RIGHT
                self.search_start = now
        elif self.search_state == SEARCH_RIGHT:
            self.stop_motors()
            if now - self.search_start > 2.4:
                print('Checked RIGHT... no line')
                self.search_state = SEARCH_BACK
                self.search_start = now
        elif self.search_state == SEARCH_BACK:
            self.stop_motors()
            if now - self.search_start > 1.0:
                print('Checked BACK... no line')
                self.search_state = SEARCH_LEFT
                self.search_start = now

    def reset_search(self):
        self.search_state = SEARCH_IDLE
        self.search_start = None

    def detect_line(self, frame):
        h, w = frame.shape[:2]
        frame_cx = w // 2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        _, binary = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)
        roi = binary[h // 2:h, :]
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        direction = 'NO LINE'
        line_cx = None
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area > 200:
                M = cv2.moments(largest)
                if M['m00'] != 0:
                    line_cx = int(M['m10'] / M['m00'])
                    line_cy = int(M['m01'] / M['m00']) + h // 2
                    offset = line_cx - frame_cx
                    if area < 1000:
                        direction = 'BACK'
                    elif offset < -30:
                        direction = 'LEFT'
                    elif offset > 30:
                        direction = 'RIGHT'
                    else:
                        direction = 'STRAIGHT'
                    cv2.circle(frame, (line_cx, line_cy), 12, (0, 255, 0), -1)
                    cv2.drawContours(frame, [largest + np.array([0, h // 2])], -1, (0, 255, 0), 2)
        cv2.line(frame, (frame_cx, 0), (frame_cx, h), (255, 0, 0), 2)
        cv2.line(frame, (0, h // 2), (w, h // 2), (0, 255, 255), 1)
        color_map = {
            'LEFT': (0, 165, 255),
            'RIGHT': (0, 165, 255),
            'STRAIGHT': (0, 255, 0),
            'BACK': (0, 0, 255),
            'NO LINE': (80, 80, 80),
        }
        cv2.putText(frame, 'Dir: ' + direction, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color_map.get(direction, (255, 255, 255)), 3)
        if line_cx is not None:
            cv2.putText(frame, 'Offset: ' + str(line_cx - frame_cx) + 'px', (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        motor_text = 'MOTORS: ON' if direction != 'NO LINE' else 'MOTORS: OFF'
        motor_color = (0, 255, 0) if direction != 'NO LINE' else (0, 0, 255)
        cv2.putText(frame, motor_text, (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, motor_color, 2)
        return frame, direction

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame capture failed')
            self.stop_motors()
            return
        annotated, direction = self.detect_line(frame)
        if direction == 'NO LINE':
            self.do_search()
        else:
            if self.search_state != SEARCH_IDLE:
                print('LINE FOUND - motors ON')
                self.reset_search()
            self.move(direction)
            if direction != self.last_direction:
                print('Direction: ' + direction)
        self.last_direction = direction
        msg = String()
        msg.data = direction
        self.dir_pub.publish(msg)
        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.img_pub.publish(img_msg)

    def destroy_node(self):
        self.stop_motors()
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
