
#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist


class TaskB(Node):

    def __init__(self, marker_id):
        super().__init__('task_b')

        # ----------------------------------------------
        # TASK STATE
        # ----------------------------------------------

        self.state = 'INITIALIZE'
        self.completed = False
        self.marker_id = marker_id

        # ----------------------------------------------
        # MARKER ID -> GOAL (x, y, yaw)
        # Coordinates are in the map frame.
        # Yaw is in radians.
        # ----------------------------------------------

        self.goals = {
            0: (3, -0.3, 0.0),
            1: (3, -0.6, 0.0),
            2: (3, -0.9, 0.0),
            3: (3, -1.1, 0.0),
        }

        # ----------------------------------------------
        # VELOCITY PUBLISHER
        # ----------------------------------------------

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ----------------------------------------------
        # STARTUP FORWARD MOVEMENT
        # ----------------------------------------------

        self.startup_forward_speed = 0.08   # m/s
        self.startup_forward_distance = 0.6 
        

        self.motion_start_time = None
        self.motion_duration = 0.0

        # ----------------------------------------------
        # NAV2 ACTION CLIENT
        # ----------------------------------------------

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.goal_handle = None
        self.result_future = None

        # ----------------------------------------------
        # TIMER
        # ----------------------------------------------

        self.timer = self.create_timer(
            0.1,
            self.update_task
        )

        self.get_logger().info(
            f'Task B initialized with ArUco ID {self.marker_id}'
        )

        # Move forward before starting navigation.
        if self.marker_id not in self.goals:
            self.get_logger().error(
                f'No goal configured for ArUco ID {self.marker_id}'
            )
            self.state = 'FAILED'
        else:
            self.start_forward()

    # ==============================================
    # TASK UPDATE
    # ==============================================

    def update_task(self):

        if self.state == 'STARTUP_FORWARD':
            self.update_motion()

        elif self.state == 'WAITING_FOR_NAV2':
            if self.nav_client.server_is_ready():
                self.send_goal()

        elif self.state == 'NAVIGATING':
            if (
                self.result_future is not None
                and self.result_future.done()
            ):
                self.process_result()

    # ==============================================
    # STARTUP FORWARD MOVEMENT
    # ==============================================


    def start_forward(self):
        self.motion_duration = (
            self.startup_forward_distance
            / self.startup_forward_speed
        )

        self.motion_start_time = self.get_clock().now()
        self.state = 'STARTUP_FORWARD'

        self.get_logger().info(
            f'Moving forward approximately '
            f'{self.startup_forward_distance:.2f} m '
            f'at {self.startup_forward_speed:.2f} m/s.'
        )


    def update_motion(self):
        if self.motion_start_time is None:
            self.stop_robot()
            self.state = 'FAILED'
            return

        elapsed = (
            self.get_clock().now() - self.motion_start_time
        ).nanoseconds / 1e9

        if elapsed < self.motion_duration:
            # Keep publishing the velocity command.
            self.publish_velocity(
                self.startup_forward_speed,
                0.0
            )
        else:
            self.stop_robot()
            self.motion_start_time = None

            self.get_logger().info(
                'Startup movement finished. Waiting for Nav2.'
            )

            self.state = 'WAITING_FOR_NAV2'


    # ==============================================
    # SEND NAVIGATION GOAL
    # ==============================================

    def send_goal(self):

        x, y, yaw = self.goals[self.marker_id]

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0

        # Convert yaw angle to quaternion.
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Navigating to marker {self.marker_id}: '
            f'x={x}, y={y}, yaw={yaw}'
        )

        self.state = 'SENDING_GOAL'

        send_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        send_future.add_done_callback(
            self.goal_response_callback
        )

    # ==============================================
    # GOAL ACCEPTANCE
    # ==============================================

    def goal_response_callback(self, future):

        try:
            self.goal_handle = future.result()

        except Exception as exc:
            self.get_logger().error(
                f'Failed to send navigation goal: {exc}'
            )
            self.state = 'FAILED'
            return

        if not self.goal_handle.accepted:
            self.get_logger().error(
                'Nav2 rejected the goal.'
            )
            self.state = 'FAILED'
            return

        self.get_logger().info(
            'Nav2 accepted the goal.'
        )

        self.result_future = (
            self.goal_handle.get_result_async()
        )

        self.state = 'NAVIGATING'

    # ==============================================
    # NAVIGATION FEEDBACK
    # ==============================================

    def feedback_callback(self, feedback_msg):

        distance = feedback_msg.feedback.distance_remaining

        self.get_logger().info(
            f'Distance remaining: {distance:.2f} m',
            throttle_duration_sec=2.0
        )

    # ==============================================
    # NAVIGATION RESULT
    # ==============================================

    def process_result(self):

        try:
            result = self.result_future.result()

        except Exception as exc:
            self.get_logger().error(
                f'Failed to retrieve navigation result: {exc}'
            )
            self.stop_robot()
            self.state = 'FAILED'
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:

            self.get_logger().info(
                f'Navigation to marker {self.marker_id} succeeded!'
            )

            self.completed = True
            self.state = 'COMPLETE'

        else:
            self.get_logger().error(
                f'Navigation failed. Status: {result.status}'
            )

            self.stop_robot()
            self.state = 'FAILED'

    # ==============================================
    # VELOCITY CONTROL
    # ==============================================

    def publish_velocity(self, linear, angular):

        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)

    # ==============================================
    # CLEANUP
    # ==============================================

    def destroy_node(self):

        self.stop_robot()

        if hasattr(self, 'timer'):
            self.timer.cancel()

        super().destroy_node()


# ==============================================
# MAIN
# ==============================================

def main(args=None):

    rclpy.init(args=args)

    # Example only.
    # Normally main_control.py passes the ArUco ID
    # detected by Task A.
    task = TaskB(marker_id=0)

    try:
        while (
            rclpy.ok()
            and task.state not in ('COMPLETE', 'FAILED')
        ):
            rclpy.spin_once(
                task,
                timeout_sec=0.1
            )

    except KeyboardInterrupt:
        pass

    finally:
        task.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
