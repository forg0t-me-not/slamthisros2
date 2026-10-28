

import cv2
import numpy as np
import time
import math
import rclpy

from Task_method import TaskMethod


class TaskA(TaskMethod):

    def __init__(self):
        super().__init__('task_a')
        
        # move state
        self.angular = 0.0
        self.linear = 0.0
        # Movement settings
        
        self.linear_speed = 0.15
        self.turn_speed = 0.5
        self.turn_duration = 1.6*((math.pi / 2.0) / self.turn_speed)

        # QR settings
        self.qr_min_width = 45
        self.qr_detector = cv2.QRCodeDetector()

        # ArUco settings
        self.goal_id = -1  # -1 means accept any ArUco marker
        self.aruco_detector = self.create_aruco_detector()

        # State machine
        self.state = 'FOLLOW_PATH'
        self.turn_start_time = None
        self.skip_aruco = True

        self.get_logger().info('Task A initialized')

    # --------------------------------------------------
    # ARUCO DETECTOR
    # --------------------------------------------------

    def create_aruco_detector(self):

        if not hasattr(cv2, 'aruco'):
            self.get_logger().error(
                'cv2.aruco is unavailable. Install opencv-contrib-python '
                'or the appropriate OpenCV contrib package.'
            )
            return None

        aruco = cv2.aruco

        dictionary = aruco.getPredefinedDictionary(
            aruco.DICT_4X4_50
        )

        if hasattr(aruco, 'DetectorParameters'):
            params = aruco.DetectorParameters()
        else:
            params = aruco.DetectorParameters_create()

        if hasattr(aruco, 'ArucoDetector'):
            detector = aruco.ArucoDetector(dictionary, params)
            return ('new', detector)

        return ('old', (dictionary, params))

    def detect_aruco(self, image):

        if self.aruco_detector is None:
            return [], None

        aruco = cv2.aruco

        if self.aruco_detector[0] == 'new':
            corners, ids, _ = self.aruco_detector[1].detectMarkers(
                image
            )
        else:
            dictionary, params = self.aruco_detector[1]

            corners, ids, _ = aruco.detectMarkers(
                image,
                dictionary,
                parameters=params
            )

        if ids is None:
            return [], None

        return [int(value) for value in ids.flatten()], corners

    # --------------------------------------------------
    # MAIN TASK
    # --------------------------------------------------

    def process_image(self, image):

        display = image.copy()

        # ----------------------------------------------
        # STATE 1: FOLLOW YELLOW PATH + DETECT QR
        # ----------------------------------------------

        if self.state == 'FOLLOW_PATH':

            qr_text, qr_points, _ = self.qr_detector.detectAndDecode(
                image
            )

            # Check whether a QR code is close enough
            qr_close = False

            if qr_points is not None and len(qr_points) > 0:

                points = qr_points[0].astype(np.int32)

                qr_width = int(
                    max(
                        np.ptp(points[:, 0]),
                        np.ptp(points[:, 1])
                    )
                )

                cv2.polylines(
                    display,
                    [points],
                    True,
                    (0, 0, 255),
                    3
                )

                if qr_width >= self.qr_min_width:
                    qr_close = True

            if qr_close:

                self.state = 'TURN'
                self.turn_start_time = time.monotonic()

                cv2.putText(
                    display,
                    'QR DETECTED - TURNING',
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )

                

                self.get_logger().info(
                    f'QR detected: {qr_text!r}. '
                    'Starting 90-degree turn.'
                )


            # Detect yellow path
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            lower_yellow = np.array([15, 75, 75], dtype=np.uint8)
            upper_yellow = np.array([40, 255, 255], dtype=np.uint8)

            mask = cv2.inRange(
                hsv,
                lower_yellow,
                upper_yellow
            )

            height, width = mask.shape

            # Use the lower 45% of the image
            roi_y = int(height * 0.55)
            roi = mask[roi_y:, :]

            moments = cv2.moments(roi)

            # Draw the line-following region
            cv2.line(
                display,
                (0, roi_y),
                (width, roi_y),
                (255, 0, 0),
                2
            )

            if moments['m00'] > 500:

                cx = int(moments['m10'] / moments['m00'])

                error = (
                    cx - width / 2.0
                ) / (width / 2.0)

                angular = max(
                    -1.0,
                    min(1.0, -1.1 * error)
                )

                linear = self.linear_speed * max(
                    0.45,
                    1.0 - 0.55 * abs(error)
                )

                # Draw detected path center
                cv2.circle(
                    display,
                    (cx, roi_y + roi.shape[0] // 2),
                    8,
                    (0, 255, 0),
                    -1
                )

                cv2.putText(
                    display,
                    'FOLLOWING YELLOW PATH',
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

            else:

                # Path missing: rotate slowly to search
                linear = 0.0
                angular = 0.22

                cv2.putText(
                    display,
                    'SEARCHING FOR PATH',
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 165, 255),
                    2
                )
                
            
            self.angular = angular
            self.linear = linear

        # ----------------------------------------------
        # STATE 2: TURN 90 DEGREES
        # ----------------------------------------------

        elif self.state == 'TURN':

            elapsed = time.monotonic() - self.turn_start_time

            cv2.putText(
                display,
                'TURNING 90 DEGREES',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )



            if elapsed < self.turn_duration:

                self.angular = -self.turn_speed
                self.linear = 0.0
            else:
                # Turn finished
                self.state = 'WAIT_ARUCO'

                self.get_logger().info(
                    'Turn finished. Waiting for ArUco marker.'
                )


        # ----------------------------------------------
        # STATE 3: WAIT FOR ARUCO MARKER
        # ----------------------------------------------

        elif self.state == 'WAIT_ARUCO':
            
            if self.skip_aruco:
                self.state = 'COMPLETE'

                self.get_logger().info(
                    'ArUco skipped. Task A complete!'
                )

            
            ids, corners = self.detect_aruco(image)

            if corners is not None:
                cv2.aruco.drawDetectedMarkers(
                    display,
                    corners,
                    np.array(ids).reshape(-1, 1)
                )

            matched_ids = (
                ids if self.goal_id < 0
                else [marker_id for marker_id in ids
                      if marker_id == self.goal_id]
            )

            if matched_ids:

                self.state = 'COMPLETE'

                cv2.putText(
                    display,
                    f'ARUCO FOUND: {matched_ids}',
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                self.get_logger().info(
                    f'ArUco marker detected: {matched_ids}. '
                    'Task complete.'
                )

            else:

                cv2.putText(
                    display,
                    f'WAITING FOR ARUCO - IDs: {ids}',
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 165, 255),
                    2
                )
            self.linear = 0.0
            self.angular = 0.0


        # ----------------------------------------------
        # STATE 4: TASK COMPLETE
        # ----------------------------------------------

        elif self.state == 'COMPLETE':

            cv2.putText(
                display,
                'TASK COMPLETE',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )
            self.linear = 0.0
            self.angular = 0.0

        self.show_images(display)
        
        return self.linear, self.angular

    # --------------------------------------------------
    # DISPLAY
    # --------------------------------------------------

    def show_images(self, display):

        cv2.imshow('Task A - Camera', display)

        cv2.waitKey(1)


# ------------------------------------------------------
# MAIN
# ------------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    task = TaskA()

    try:
        task.get_logger().info(
            'Task A started: yellow path -> QR -> turn -> ArUco'
        )

        rclpy.spin(task)

    except KeyboardInterrupt:
        pass

    finally:
        task.stop_robot()
        task.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
