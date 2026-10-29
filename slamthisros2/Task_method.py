
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class TaskMethod(Node):

    def __init__(self, node_name='task_controller'):
        super().__init__(node_name)

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        image_topic = self.get_parameter('image_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.bridge = CvBridge()

        # Central velocity publisher
        self.cmd_pub = self.create_publisher(
            Twist, cmd_vel_topic, 10
        )

        # Central camera subscriber
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f'{node_name} ready')

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            # Run the active task's behavior
            linear, angular = self.process_image(image)

            # Publish its velocity
            self.publish_velocity(linear, angular)

        except Exception as exc:
            self.get_logger().error(f'Task error: {exc}')
            self.stop_robot()

    def process_image(self, image):
        """
        Child classes must implement their own task behavior.
        Return: (linear_velocity, angular_velocity)
        """
        raise NotImplementedError(
            'Child task must implement process_image()'
        )

    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()