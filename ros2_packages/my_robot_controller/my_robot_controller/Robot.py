import cv2
import rclpy
import time
from colorama import Fore, Style
import random

from geometry_msgs.msg import Twist
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from .Detector import Detector
# import threading


YOLO_FPS = 4
PROCESS_FREQUENCY = 100


class StateMachine:
    def __init__(self, states):
        self.states = states
        self.current_state = states[0]
        self.name_to_index = {name: i for i, name in enumerate(states)}

    def increase_state(self):
        index = self.name_to_index[self.current_state]
        next_index = (index + 1) % len(self.states)
        self.current_state = self.states[next_index]

    def decrease_state(self):
        index = self.name_to_index[self.current_state]
        prev_index = (index - 1) % len(self.states)
        self.current_state = self.states[prev_index]

    def set_state(self, state):
        if state not in self.states:
            raise Exception(f"You are trying to set incorrect state {state}")
        index = self.name_to_index[state]
        self.current_state = self.states[index]

    def set_next_state(self, state):
        """ Replace state with yours if it is next.

        Args:
            state (str): State u can try to set

        Returns:
            Bool: If it really next state for current it will be set and return True else return False
        """
        index = self.name_to_index[state]
        if self.name_to_index[self.current_state] + 1 == index:
            self.increase_state()
            return True
        return False

    def get_state(self):
        return self.current_state


class Robot(Node):
    def __init__(self, mode: str = 'auto', state: str = None, camera=None, odom=None, twist=None):
        super().__init__("moving_robot")

        # Аргумент изначально будет приходить из параметров запуска, чтобы можно было управлять вручную
        modes = [
            'auto',
            'manual',
        ]
        if mode not in modes:
            mode = 'auto'

        # Создаем машину состояний состояющую из препятствий
        self.state_machine = StateMachine(
            ['traffic_light', 'crossroad', 'walls', 'parking', 'crossing', 'tunnel'])
        if state is not None:
            self.state_machine.set_state(state)

        self.obstacles = {
            'tunnel': TunnelObstacle()
        }

        self.detector = Detector()
        self.lane_follow = LaneFollowing()
        self.not_finished = True
        self.mode = None

        self.error = None

        self.position = None
        self.orientation = None

        self.linear_velocity = None
        self.angular_velocity = None

        self.frame = None
        self.cv_image = None
        self.depth_image = None
        self.lidar_scan = None
        self.yolo_result = None
        self.yolo_image = None

        self.bridge = CvBridge()

        self.create_subscription(
            Image, '/color/image', self._camera_callback, 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_subscription(
            Image,
            '/depth/image',
            self._depth_callback,
            10)
        self.create_subscription(
            LaserScan,
            '/scan',
            self._lidar_callback,
            10
        )

        # self.detector_thread = threading.Thread(target=self.run_detector, daemon=True)
        # self.detector_thread.start()

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Таймер для управления роботом (движение по линии / препятствиям и тд)
        self.create_timer(1.0 / PROCESS_FREQUENCY, self.process_mode)
        # Таймер для детектора, по которому закидываются картинки
        self.create_timer(1.0 / YOLO_FPS, self.run_detector)

        self.get_logger().info('Robot interface initialized')
    """
    This block is for sensors callback
    Just look to next block for aviable functions
    """

    def _lidar_callback(self, msg):
        try:
            self.lidar_scan = msg
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def _depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def _camera_callback(self, msg):
        try:
            self.frame = msg
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert color image: {e}")

    def _odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.linear_velocity = msg.twist.twist.linear
        self.angular_velocity = msg.twist.twist.angular
    """"
    This block returns values from all sensors (odometry too)
    """

    def get_lidar(self):
        return self.lidar_scan

    def get_depth(self):
        return self.depth_image

    def get_image(self):
        return self.cv_image

    def get_odometry(self):
        return {
            'position': self.position,
            'orientation': self.orientation,
            'linear_velocity': self.linear_velocity,
            'angular_velocity': self.angular_velocity,
        }

    """
    This block of code is for your code and etc.
    Just work here and don't care
    """

    def move(self, linear_x=0.0, angular_z=0.0):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_publisher.publish(cmd)
        # self.get_logger().info(f"Command sent: linear_x={linear_x}, angular_z={angular_z}")

    # Main function
    def process_mode(self):
        # start_time = time.time()
        if self.state_machine.get_state == 'tunnel':
            self.obstacles['tunnel'].process()
        elif self.cv_image is not None:
            self.lane_follow.just_follow(self)
        # if random.randint(0, 10) == 1:
            # self.get_logger().info(f"Lane was processed for {time.time() - start_time}s")

    def run_detector(self) -> None:
        # Запускает детектор, который получает боксы из YOLO в виде списка словарей, посмотреть структуру можно внутри функции
        try:
            if self.cv_image is not None:
                # TODO: сделать блокировку
                boxes, self.yolo_image, duration = self.detector.process_image(self.cv_image)
                self.check_for_state_transition(boxes)

                # logging
                # self.get_logger().info(
                # f"Image was processed with YOLO for {duration} seconds")
                # for box in boxes:
                    # self.get_logger().info(f"Found {box['label']} with {box['conf']} conf and {box['area']} area")
        except Exception as e:
            self.get_logger().error(
                f"Robot.run_detector exception: {Fore.RED}{e}{Style.RESET_ALL}")

    def check_for_state_transition(self, boxes):
        # Возмонжо стоит поменять, запускается из детектора и пытается перейти к следующему состоянию по условию
        for box in boxes:
            # if box['conf'] > 0.85:
                # self.get_logger().info(f"Found {box['label']} with {box['conf']} conf and {box['area']} area")
            if box['label'] == 'tunnel_sign' and box['area'] > 10000.0 and box['conf'] > 0.85:# and self.state_machine.set_next_state('tunnel'):
                self.get_logger().info("States was updated to tunnel by")
                self.get_logger().info(f"{box['conf']} conf and {box['area']} area")


# Class for lane following code
class LaneFollowing():
    def __init__(self, speed=0.15, h=None, w=None):
        h_real = h//4 if h else 212
        w_real = w//6 if w else 60

        self.prevpt1, self.prevpt2 = (h_real, w_real), (h_real*3, w_real)
        self.speed = speed

    def just_follow(self, robot: Robot):
        # Конвертация сообщения ROS в OpenCV изображение
        image = robot.cv_image
        condition = ((image[:, :, 2] > 220) & (
            image[:, :, 1] > 220) & (image[:, :, 0] < 30))
        image[condition] = 255
        grays = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Увеличение контраста и бинаризация
        grays[grays < 219] = 0
        # grayd = grays + 100 - np.mean(grays)/3
        _, gray = cv2.threshold(grays, 170, 255, cv2.THRESH_BINARY)

        # Обработка нижней трети изображения
        height, width = gray.shape
        dst = gray[2 * height // 3:, :]
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        dst = cv2.morphologyEx(dst, cv2.MORPH_CLOSE, kernel)
        # Нахождение компонентов
        num_labels, _, _, centroids = cv2.connectedComponentsWithStats(dst)
        dst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        if num_labels > 1:
            mindistance1 = []
            mindistance2 = []

            for i in range(1, num_labels):
                cv2.circle(dst, (int(centroids[i][0]), int(
                    centroids[i][1])), 2, (0, 255, 0), 2)
                centroid = centroids[i]

                # Расчёт расстояний до предыдущих точек
                distance1 = abs(
                    centroid[0] - (self.prevpt1[0] if self.prevpt1 else 0))
                distance2 = abs(
                    centroid[0] - (self.prevpt2[0] if self.prevpt2 else 0))

                mindistance1.append(distance1)
                mindistance2.append(distance2)

            # Поиск минимальных расстояний
            min_dist1 = min(mindistance1)
            min_dist2 = min(mindistance2)

            min_idx1 = mindistance1.index(min_dist1) + 1
            min_idx2 = mindistance2.index(min_dist2) + 1
            print(min_dist1)
            # Определение новых точек
            cpt1 = tuple(centroids[min_idx1]
                         ) if min_dist1 < 100 else self.prevpt1
            cpt2 = tuple(centroids[min_idx2]
                         ) if min_dist2 < 100 else self.prevpt2
        else:
            # Если нет объектов, используем предыдущие точки
            cpt1, cpt2 = self.prevpt1, self.prevpt2

        # Сохранение текущих точек
        self.prevpt1, self.prevpt2 = cpt1, cpt2

        # Вычисление центральной точки
        if cpt1 and cpt2:
            fpt_x = int((cpt1[0] + cpt2[0]) / 2)
            fpt_y = int((cpt1[1] + cpt2[1]) / 2) + 2 * height // 3
        else:
            fpt_x, fpt_y = width // 2, height

        self.error = width // 2 - fpt_x

        # Движение

        robot.move(linear_x=self.speed, angular_z=(
            self.error * 90.0 / 400) / 15)
        # Отображение результатов

        if cpt1 and cpt2:
            cv2.circle(image, (fpt_x, fpt_y), 2, (0, 0, 255), 2)
            cv2.circle(dst, (int(cpt1[0]), int(cpt1[1])), 2, (0, 0, 255), 2)
            cv2.circle(dst, (int(cpt2[0]), int(cpt2[1])), 2, (255, 0, 0), 2)

        # Выводит склееное вертикально изображение из yolo и нахождение центра масс
        if robot.yolo_image is not None and dst is not None:
            concatenated_image = np.vstack((robot.yolo_image, dst))
        # то же что и выше но картинка не после yolo а оригинальная
        # if image is not None and dst is not None:
        #     concatenated_image = np.vstack((image, dst))
            cv2.imshow('Camera', concatenated_image)
            cv2.waitKey(1)


# Класс-родитель препятствие в каждом из которых в process будет реализована логика прохождения препятствия
class Obstacle:
    def __init__(self, name: str):
        self.name = name

    def process(self, robot: Robot):
        return None

# Класс препятствия для туннеля
class TunnelObstacle(Obstacle):
    def __init__(self):
        super().__init__('tunnel')

    def process(self, robot: Robot):
        robot.move(linear_x=0.01)
        robot.get_logger().info(f"lidar shape: {robot.get_lidar()}")
