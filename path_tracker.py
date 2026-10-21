#!/usr/bin/env python3
"""Camera-based yellow-line follower with QR stop, 90-degree turn, and ArUco wait."""
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FinalController(Node):
    def __init__(self):
        super().__init__('final_controller')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('goal_id', -1)  # -1 accepts any detected ArUco ID
        self.declare_parameter('linear_speed', 0.07)
        self.declare_parameter('turn_speed', 0.45)
        self.declare_parameter('turn_seconds', math.pi / 2.0 / 0.45)
        self.declare_parameter('qr_min_width', 65)

        image_topic = self.get_parameter('image_topic').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.goal_id = int(self.get_parameter('goal_id').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.turn_seconds = float(self.get_parameter('turn_seconds').value)
        self.qr_min_width = int(self.get_parameter('qr_min_width').value)

        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.state = 'FOLLOW_PATH'
        self.turn_started = None
        self.last_image_time = time.monotonic()
        self.last_log_time = 0.0
        self.qr_detector = cv2.QRCodeDetector()
        self.aruco_detector = self._make_aruco_detector()
        self.get_logger().info(f'Controller ready. Image={image_topic}, cmd_vel={cmd_topic}, goal_id={self.goal_id}')

    def _make_aruco_detector(self):
        # ArUco requires opencv-contrib-python / system OpenCV with cv2.aruco.
        if not hasattr(cv2, 'aruco'):
            self.get_logger().error('cv2.aruco is missing. Install an OpenCV contrib package before running.')
            return None
        aruco = cv2.aruco
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        if hasattr(aruco, 'DetectorParameters'):
            params = aruco.DetectorParameters()
        else:  # compatibility with older OpenCV
            params = aruco.DetectorParameters_create()
        if hasattr(aruco, 'ArucoDetector'):
            return ('new', aruco.ArucoDetector(dictionary, params))
        return ('old', (dictionary, params))

    def publish_velocity(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def detect_aruco_ids(self, image):
        if self.aruco_detector is None:
            return []
        aruco = cv2.aruco
        try:
            if self.aruco_detector[0] == 'new':
                _, ids, _ = self.aruco_detector[1].detectMarkers(image)
            else:
                dictionary, params = self.aruco_detector[1]
                _, ids, _ = aruco.detectMarkers(image, dictionary, parameters=params)
            return [] if ids is None else [int(v) for v in ids.flatten()]
        except Exception as exc:
            self.get_logger().warning(f'ArUco detection error: {exc}')
            return []

    def image_callback(self, msg):
        self.last_image_time = time.monotonic()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Cannot convert camera image: {exc}')
            self.publish_velocity()
            return

        height, width = frame.shape[:2]
        now = time.monotonic()

        if self.state == 'FOLLOW_PATH':
            # Detect the STOP QR code. Treat any readable QR as the stop sign.
            qr_text, qr_points, _ = self.qr_detector.detectAndDecode(frame)
            qr_close = False
            if qr_points is not None and len(qr_points) > 0:
                pts = qr_points[0].astype(np.int32)
                qr_width = int(max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1])))
                qr_close = qr_width >= self.qr_min_width
            if qr_close:
                self.state = 'TURN_90'
                self.turn_started = now
                self.publish_velocity()
                self.get_logger().info(f'QR detected (text={qr_text!r}, width threshold met). Stopping and turning 90 degrees.')
                return

            # Yellow path segmentation in HSV. Tune these thresholds if the track lighting differs.
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([15, 75, 75], dtype=np.uint8)
            upper_yellow = np.array([40, 255, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            roi_y = int(height * 0.55)
            roi = mask[roi_y:, :]
            moments = cv2.moments(roi)
            if moments['m00'] > 500:
                cx = int(moments['m10'] / moments['m00'])
                error = (cx - width / 2.0) / (width / 2.0)
                angular = max(-1.0, min(1.0, -1.1 * error))
                speed = self.linear_speed * max(0.45, 1.0 - 0.55 * abs(error))
                self.publish_velocity(speed, angular)
                self._log_state(f'Yellow path: center={cx}, error={error:.2f}')
            else:
                # Do not drive blindly if the line disappears. Search slowly in place.
                self.publish_velocity(0.0, 0.22)
                self._log_state('Yellow path not visible; searching slowly')

        elif self.state == 'TURN_90':
            elapsed = now - self.turn_started
            if elapsed < self.turn_seconds:
                self.publish_velocity(0.0, -self.turn_speed)
            else:
                self.publish_velocity()
                self.state = 'WAIT_ARUCO'
                self.get_logger().info('90-degree turn timer finished. Stopped; waiting for ArUco marker.')

        elif self.state == 'WAIT_ARUCO':
            self.publish_velocity()
            ids = self.detect_aruco_ids(frame)
            matched = ids if self.goal_id < 0 else [i for i in ids if i == self.goal_id]
            if matched:
                self.state = 'COMPLETE'
                self.publish_velocity()
                self.get_logger().info(f'Goal ArUco marker detected: {matched}. Task complete; robot stopped.')
            else:
                self._log_state(f'Waiting for ArUco ID {self.goal_id}; visible IDs={ids}')

        elif self.state == 'COMPLETE':
            self.publish_velocity()

    def _log_state(self, message):
        now = time.monotonic()
        if now - self.last_log_time > 2.0:
            self.get_logger().info(message)
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = FinalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_velocity()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
