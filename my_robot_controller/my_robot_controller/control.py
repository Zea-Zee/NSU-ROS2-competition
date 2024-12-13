import rclpy
import cv2
import keyboard
import time
import pygame

from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge


class Detector():
    def __init__(self):
        self.net = None



class Robot(Node):
    def __init__(self, camera=None, odom=None, twist=None):
        super().__init__("moving_robot")

        self.detector = Detector()

        self.not_finished = True
        self.mode = None

        self.position = None
        self.orientation = None

        self.linear_velocity = None
        self.angular_velocity = None

        self.cv_image = None

        self.bridge = CvBridge()

        self.create_subscription(Image, '/camera/image_raw', self._camera_callback, 10)

        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.1, self.process_mode)


        self.get_logger().info('Robot interface initialized')


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
        
    def process_mode(self):
        image = self.cv_image


class WASDController:
    def __init__(self, robot):
        self.robot = robot
        self.linear_speed = 0.2  # Скорость движения вперёд/назад
        self.angular_speed = 0.5  # Скорость поворота

        # Инициализация pygame
        pygame.init()
        self.screen = pygame.display.set_mode((200, 200))  # Маленькое окно для фокуса
        pygame.display.set_caption("WASD Controller")

        self.robot.get_logger().info("WASD controller initialized with pygame.")
        self.robot.get_logger().info("Use WASD keys to control the robot. Close the window to quit.")

    def handle_input(self):
        """Обрабатывает события pygame и отправляет команды роботу."""
        keys = pygame.key.get_pressed()

        if keys[pygame.K_w]:  # Движение вперёд
            self.robot.move(self.linear_speed, 0.0)
        elif keys[pygame.K_s]:  # Движение назад
            self.robot.move(-self.linear_speed, 0.0)
        elif keys[pygame.K_a]:  # Поворот влево
            self.robot.move(0.0, self.angular_speed)
        elif keys[pygame.K_d]:  # Поворот вправо
            self.robot.move(0.0, -self.angular_speed)
        else:
            # Если ни одна клавиша не нажата, робот останавливается
            self.robot.move(0.0, 0.0)
    def check_exit(self):
        """Проверяет, нужно ли завершить программу."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    controller = WASDController(robot)

    frequency = 10  # Частота обработки (Гц)
    period = 1 / frequency  # Период обработки (сек)
    
    try:
        while rclpy.ok() and robot.not_finished:
            rclpy.spin_once(robot, timeout_sec=0.1)

            image = robot.get_image()
            if image is not None:
                # Отображение изображения для тестирования
                cv2.imshow('Robot Camera', image)
                cv2.waitKey(1)

            # Обработка одометрии
            odometry = robot.get_odometry()
            if odometry['position']:
                robot.get_logger().info(f"Robot position: {odometry['position']}")
            rclpy.spin_once(robot, timeout_sec=0.1)

            if controller.check_exit():
                robot.get_logger().info("Exiting WASD controller...")
                break

            controller.handle_input()

    except KeyboardInterrupt:
        robot.get_logger().info('Shutting down robot node...')
    finally:
        robot.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
