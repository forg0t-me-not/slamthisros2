#!/usr/bin/env python3
"""Task A: two-point yellow tracker, saved-cyan close-range fallback, timed clockwise turn."""
import cv2
import numpy as np
import time
import math
import rclpy

from .Task_method import TaskMethod
from pyzbar.pyzbar import decode as zbar_decode, ZBarSymbol


class TaskA(TaskMethod):

    def __init__(self):
        super().__init__('task_a')

        # move state
        self.angular = 0.0
        self.linear = 0.0
        # Movement settings

        self.linear_speed = 0.13
        self.turn_speed = 0.4
        self.turn_duration = (math.pi / 2.0) / self.turn_speed
        self.stop_pause_seconds = 2.0  # stationary pause after QR STOP, before turn
        self.stop_pause_started = None

        # QR settings
        self.declare_parameter('qr_trigger_ratio', 0.265)
        self.qr_trigger_ratio = float(self.get_parameter('qr_trigger_ratio').value)
        self.stop_decoded = False
        self.stop_last_seen = None
        # Line-tracking memory: rejects sudden jumps to another yellow section.
        self.prev_near_cx = None
        self.prev_far_cx = None
        # Retain the LAST ACCEPTED cyan point even when the line leaves view.
        # A frozen image coordinate is only useful for a brief approach;
        # it is not a continuously measured physical target.
        self.saved_cyan_cx = None
        self.saved_cyan_at = None
        self.qr_slow_speed = 0.08
        # The perspective camera may lose the line before the robot reaches
        # the QR-size trigger. Only after STOP is decoded, make a short,
        # low-speed approach guided briefly by the last accepted cyan x.
        self.no_line_qr_speed = 0.035            # m/s
        self.no_line_qr_max_seconds = 2.5      # limit motion with no line
        self.no_line_qr_min_ratio = 0.18       # must already be near STOP
        self.no_line_qr_fresh_seconds = 1.0    # latest QR decode is recent
        self.no_line_qr_since = None
        self.saved_cyan_kp = 0.20        # P steering on frozen normalized pixel error
        self.saved_cyan_max_w = 0.055  # rad/s; avoid large CCW/CW corrections
        self.last_stop_ratio = None
        self.last_no_line_log = 0.0
        self._window_closed = False
        # Open-loop 90-degree clockwise turn: duration = angle / speed.
        # No odometry subscription or feedback controller in Task A.

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
            aruco.DICT_5X5_50
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
    # CONTINUITY-AWARE TWO-POINT YELLOW TRACKER
    # --------------------------------------------------

    @staticmethod
    def _yellow_runs(mask, center_y, half_height=3):
        """Return horizontal runs of yellow pixels in a narrow row strip."""
        height, width = mask.shape
        y0 = max(0, int(center_y) - half_height)
        y1 = min(height, int(center_y) + half_height + 1)
        if y1 <= y0:
            return []
        # Require support from several rows to avoid isolated yellow noise.
        row_votes = np.count_nonzero(mask[y0:y1] > 0, axis=0)
        active = row_votes >= min(3, y1 - y0)
        padded = np.r_[False, active, False]
        starts = np.flatnonzero(np.diff(padded.astype(np.int8)) == 1)
        ends = np.flatnonzero(np.diff(padded.astype(np.int8)) == -1)
        return [(float((start + end - 1) / 2.0), int(end - start))
                for start, end in zip(starts, ends) if end - start >= 3]

    def _find_line_centers(self, mask):
        """Green: near centroid. Cyan: trace that SAME line upward row by row.

        No global yellow-pixel centroid is used for the look-ahead point.
        Each row follows a nearby yellow run, so another distant section of
        the winding track cannot suddenly become the cyan point.
        """
        height, width = mask.shape
        near_y0 = int(height * .75)
        far_y = int(height * .605)
        near_y = int(height * .87)

        # Preserve the previously working near-point measurement.
        near_band = mask[near_y0:height]
        moments = cv2.moments(near_band)
        near = (float(moments['m10'] / moments['m00'])
                if moments['m00'] > 500 else None)
        if near is None:
            self.prev_near_cx = None
            self.prev_far_cx = None
            return None, None

        # Walk from the bottom (green) to the look-ahead height (cyan).
        # A jump to a separate far-away yellow patch is not allowed.
        cursor = near
        max_step = max(18.0, width * .105)
        vertical_step = max(3, int(height * .030))
        valid_steps = 0
        for y in range(near_y, far_y - 1, -vertical_step):
            runs = self._yellow_runs(mask, y)
            candidates = [(abs(x - cursor), x) for x, length in runs
                          if abs(x - cursor) <= max_step]
            if not candidates:
                break
            cursor = min(candidates)[1]
            valid_steps += 1

        expected_steps = len(range(near_y, far_y - 1, -vertical_step))
        far = cursor if valid_steps >= max(2, expected_steps - 1) else None

        # Temporal outlier rejection: the tracked point cannot teleport.
        if (far is not None and self.prev_far_cx is not None
                and self.prev_near_cx is not None):
            expected = self.prev_far_cx + (near - self.prev_near_cx)
            if abs(far - expected) > max(24.0, width * .10):
                far = None

        self.prev_near_cx = near
        if far is not None:
            self.prev_far_cx = far
        return near, far

    # --------------------------------------------------
    # MAIN TASK
    # --------------------------------------------------

    def process_image(self, image):
        display = image.copy()

        # -------------------------------------------------------------
        # Follow yellow line until STOP QR reaches the required size.
        # Once STOP is first decoded, keep line-following (not blind
        # straight driving), but do not use an unrelated cyan position.
        # -------------------------------------------------------------
        if self.state == 'FOLLOW_PATH':
            now = time.monotonic()
            height, width = image.shape[:2]
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            decoded = zbar_decode(gray, symbols=[ZBarSymbol.QRCODE])
            stop_ratio = None
            for qr in decoded:
                text = qr.data.decode('utf-8', errors='replace').strip()
                rect = qr.rect
                x, y, w, h = rect.left, rect.top, rect.width, rect.height
                ratio = max(w, h) / width
                cv2.rectangle(display, (x, y), (x + w, y + h),
                              (0, 0, 255), 2)
                if text.upper() != 'STOP':
                    continue
                stop_ratio = ratio if stop_ratio is None else max(stop_ratio, ratio)
                self.stop_last_seen = now
                self.last_stop_ratio = ratio
                if not self.stop_decoded:
                    self.stop_decoded = True
                    self.get_logger().info(
                        f'STOP decoded ratio={ratio:.3f}; '
                        'continuing yellow line tracking to QR size threshold')
                self.get_logger().info(
                    f'QR STOP ratio={ratio:.3f}', throttle_duration_sec=1.0)

            # Do not stop on first sight. Stop ONLY at the required QR size.
            if stop_ratio is not None and stop_ratio >= self.qr_trigger_ratio:
                self.linear = self.angular = 0.0
                self.state = 'STOP_PAUSE'
                self.stop_pause_started = now
                self.get_logger().info(
                    f'STOP size reached ratio={stop_ratio:.3f} '
                    f'>= {self.qr_trigger_ratio:.3f}; '
                    f'holding position for {self.stop_pause_seconds:.1f}s before turning')
                cv2.putText(display, 'STOP - PAUSE BEFORE TURN', (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 255), 2)
                self.show_images(display)
                return 0.0, 0.0

            # A QR can leave the camera view temporarily on a curved track.
            # Once STOP is decoded, remember it, but do not abort while the
            # yellow line is still trackable. The separate no-line fallback
            # below imposes its own time limit and requires a fresh, close QR.

            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([15, 75, 75], np.uint8),
                              np.array([40, 255, 255], np.uint8))
            near_cx, far_cx = self._find_line_centers(mask)
            if far_cx is not None:
                # Spatial and temporal consistency were checked by the tracker.
                self.saved_cyan_cx = float(far_cx)
                self.saved_cyan_at = now
            near_top = int(height * .75)
            far_y = int(height * .605)
            cv2.line(display, (0, near_top), (width, near_top), (255, 0, 0), 2)
            cv2.line(display, (0, far_y), (width, far_y), (255, 0, 0), 2)

            if near_cx is not None:
                near_error = (near_cx - width / 2.0) / (width / 2.0)
                # Only use cyan if it is spatially and temporally consistent.
                if far_cx is not None:
                    far_error = (far_cx - width / 2.0) / (width / 2.0)
                    steering_error = .72 * near_error + .28 * far_error
                    cv2.circle(display, (int(far_cx), far_y),
                               6, (255, 255, 0), -1)
                else:
                    # Look-ahead was missing/rejected: follow green only.
                    steering_error = near_error
                    cv2.putText(display, 'LOOKAHEAD REJECTED', (20, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 165, 255), 1)

                angular = float(np.clip(-1.1 * steering_error, -1.0, 1.0))
                linear = self.linear_speed * max(
                    .40, 1.0 - .60 * abs(steering_error))
                if self.stop_decoded:
                    linear = min(linear, self.qr_slow_speed)
                    if stop_ratio is not None and stop_ratio > .20:
                        linear = min(linear, .05)
                cv2.circle(display, (int(near_cx), int(height * .87)),
                           8, (0, 255, 0), -1)
                text = ('QR VISIBLE - FOLLOWING LINE' if self.stop_decoded
                        else 'FOLLOWING YELLOW PATH')
                cv2.putText(display, text, (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 0), 2)
                # Green is visible, so do not use the blind-approach timer.
                self.no_line_qr_since = None
            else:
                # The physical line still exists; it has simply gone outside
                # the 90-degree camera field of view. After STOP is decoded,
                # keep the LAST valid cyan position as a short-horizon target.
                # Stop precisely when the live QR ratio reaches the threshold.
                if self.stop_decoded:
                    if self.no_line_qr_since is None:
                        self.no_line_qr_since = now
                    lost_for = now - self.no_line_qr_since
                    qr_fresh = (self.stop_last_seen is not None
                                and now - self.stop_last_seen < self.no_line_qr_fresh_seconds)
                    qr_near = (self.last_stop_ratio is not None
                               and self.last_stop_ratio >= self.no_line_qr_min_ratio)

                    if lost_for > self.no_line_qr_max_seconds:
                        self.linear = self.angular = 0.0
                        self.state = 'ERROR'
                        self.get_logger().error(
                            'Line out of camera too long before QR reached size; halted')
                        self.show_images(display)
                        return 0.0, 0.0

                    linear = self.no_line_qr_speed if (qr_fresh and qr_near) else 0.0
                    angular = 0.0
                    if linear > 0.0 and self.saved_cyan_cx is not None:
                        # Freeze the last ACCEPTED cyan pixel x, and steer
                        # gently toward it. Fade correction near QR threshold
                        # because there is no new cyan visual feedback.
                        saved_err = ((self.saved_cyan_cx - width / 2.0)
                                     / (width / 2.0))
                        remaining_ratio = max(0.0, self.qr_trigger_ratio - self.last_stop_ratio)
                        fade = min(1.0, remaining_ratio / 0.05)
                        angular = float(np.clip(
                            -self.saved_cyan_kp * saved_err * fade,
                            -self.saved_cyan_max_w, self.saved_cyan_max_w))
                        cv2.circle(display, (int(self.saved_cyan_cx), far_y),
                                   7, (255, 255, 0), 2)
                        label = 'SAVED CYAN - QR APPROACH'
                    else:
                        label = ('QR VISIBLE - WAIT CYAN' if linear > 0.0
                                 else 'LINE OUT OF VIEW - WAIT QR')
                    if now - self.last_no_line_log > 1.0:
                        self.get_logger().info(
                            f'{label}: ratio={self.last_stop_ratio!r} '
                            f'trigger={self.qr_trigger_ratio:.3f} '
                            f'cyan_x={self.saved_cyan_cx!r} '
                            f'v={linear:.3f} w={angular:+.3f} '
                            f'line_lost={lost_for:.1f}s')
                        self.last_no_line_log = now
                else:
                    # Before STOP, retain the earlier path-search behavior.
                    linear, angular = 0.0, 0.22
                    label = 'SEARCHING FOR PATH'
                cv2.putText(display, label, (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, .52, (0, 165, 255), 2)

            self.linear = linear
            self.angular = angular

        # ----------------------------------------------
        # STATE 2: STOP AND PAUSE BEFORE TURNING
        # ----------------------------------------------

        elif self.state == 'STOP_PAUSE':
            # Zero linear AND angular velocity while stopped at the QR.
            self.linear = 0.0
            self.angular = 0.0
            elapsed = time.monotonic() - self.stop_pause_started
            if elapsed >= self.stop_pause_seconds:
                self.turn_start_time = time.monotonic()
                self.state = 'TURN'
                self.get_logger().info(
                    f'STOP pause complete ({elapsed:.2f}s); beginning clockwise turn')
            else:
                cv2.putText(display,
                            f'STOPPED {elapsed:.1f}/{self.stop_pause_seconds:.1f}s',
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                            .65, (0, 0, 255), 2)

        # ----------------------------------------------
        # STATE 3: TURN 90 DEGREES
        # ----------------------------------------------

        elif self.state == 'TURN':
            self.linear = 0.0
            elapsed = time.monotonic() - self.turn_start_time
            if elapsed < self.turn_duration:
                # ROS convention: negative angular.z turns clockwise.
                self.angular = -self.turn_speed
                cv2.putText(display,
                            f'CLOCKWISE TURN {elapsed:.1f}/{self.turn_duration:.1f}s',
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                            .65, (0, 0, 255), 2)
            else:
                self.angular = 0.0
                self.state = 'WAIT_ARUCO'
                self.get_logger().info(
                    f'Timed clockwise turn complete ({elapsed:.2f}s)')

        # ----------------------------------------------
        # STATE 3: WAIT FOR ARUCO MARKER
        # ----------------------------------------------

        elif self.state == 'WAIT_ARUCO':

            if self.skip_aruco:
                self.linear = self.angular = 0.0
                self.state = 'COMPLETE'
                self.close_camera()
                self.get_logger().info('ArUco skipped. Task A complete!')
                return 0.0, 0.0


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

        elif self.state == 'ERROR':
            self.linear = self.angular = 0.0
            self.close_camera()

        elif self.state == 'COMPLETE':
            self.close_camera()
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

        if self.state not in ('COMPLETE', 'ERROR'):
            self.show_images(display)
        return self.linear, self.angular

    # --------------------------------------------------
    # DISPLAY
    # --------------------------------------------------

    def show_images(self, display):
        if not self._window_closed:
            cv2.imshow('Task A - Camera', display)
            cv2.waitKey(1)

    def close_camera(self):
        if self._window_closed:
            return
        self._window_closed = True
        try:
            cv2.destroyWindow('Task A - Camera')
            cv2.waitKey(1)
        except cv2.error:
            pass

    def destroy_node(self):
        self.close_camera()
        return super().destroy_node()


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

        while rclpy.ok() and task.state not in ('COMPLETE', 'ERROR'):
            rclpy.spin_once(task, timeout_sec=0.1)
        task.get_logger().info(f'Task A finished: {task.state}')

    except KeyboardInterrupt:
        pass

    finally:
        task.stop_robot()
        task.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
