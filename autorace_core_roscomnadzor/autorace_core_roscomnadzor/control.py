import rclpy
import cv2
import keyboard
import time
import pygame
import os

from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from .Robot import Robot
from .WASDMove import WASDController


def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    #robot = Robot(state='crossing')
    # controller = WASDController(robot)

    frequency = 10  # Частота обработки (Гц)
    period = 1 / frequency  # Период обработки (сек)

    try:
        rclpy.spin(robot)
        # while True:
        # cv2.imshow('Robot Camera', robot.get_image())
        # cv2.waitKey(1)
        # if robot.frame:
        # robot.lane_following()
        # image = robot.get_image()
        # if image is not None:
        #     # Отображение изображения для тестирования
        #     cv2.imshow('Robot Camera', image)
        #     cv2.waitKey(1)

        # # Обработка одометрии
        # odometry = robot.get_odometry()
        # if odometry['position']:
        #     robot.get_logger().info(f"Robot position: {odometry['position']}")
        # rclpy.spin_once(robot, timeout_sec=0.1)

        # if controller.check_exit():
        #     robot.get_logger().info("Exiting WASD controller...")
        #     break

        # controller.handle_input()

    except KeyboardInterrupt:
        robot.get_logger().info('Shutting down robot node...')
    finally:
        robot.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
