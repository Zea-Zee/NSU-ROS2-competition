import cv2
from geometry_msgs.msg import Twist
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import rclpy

class Robot(Node):
    def __init__(self, camera=None, odom=None, twist=None):
        super().__init__("moving_robot")

        self.not_finished = True
        self.mode = None

        self.position = None
        self.angle = None

        self.linear_velocity = None
        self.angular_velocity = None

        self.cv_image = None

        self.bridge = CvBridge()

        self.create_subscription(Image, '/camera/image_raw', self._camera_callback, 10)

        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.1, self.process_mode)


        self.get_logger().info('Robot interface initialized')

    def process_mode(self):
        image = self.cv_image
        # Реализовать логику для выбора режимов: светофор, развилка, парковка и другое

    def _camera_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def _odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.linear_velocity = msg.twist.twist.linear
        self.angular_velocity = msg.twist.twist.angular

    def get_image(self):
        return self.cv_image

    def get_odometry(self):
        return {
            'position': self.position,
            'orientation': self.orientation,
            'linear_velocity': self.linear_velocity,
            'angular_velocity': self.angular_velocity,
        }

    def move(self, linear_x=0.0, angular_z=0.0):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info(f"Command sent: linear_x={linear_x}, angular_z={angular_z}")
