import cv2
from geometry_msgs.msg import Twist
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import rclpy

class Detector():
    def __init__(self):
        self.net = None