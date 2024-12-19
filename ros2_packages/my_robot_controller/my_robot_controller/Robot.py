import cv2
import rclpy
import time
from colorama import Fore, Style, init as colorama_init
import random
import math

from geometry_msgs.msg import Twist
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from .Detector import Detector
import time
# import threading


YOLO_FPS = 2
PROCESS_FREQUENCY = 100


def log(node: Node, message: str, log_type: str='-'):
    """Кастомный логгер.

    Args:
        node (Node): Нода из которой будет сообщение
        message (str): Сообщение
        log_type (str, optional): Тип сообщения (цвет и сообщение в росе) (INFO, GOOD_INFO, WARN, BIG_WARN, ERROR, CRITICAL_ERROR)

    Returns:
        int: 1 if was not success else 0
    """
    match log_type:
        case 'INFO':
            node.get_logger().info(f"{Fore.GREEN}{message}{Style.RESET_ALL}")
        case 'GOOD_INFO':
            node.get_logger().info(
                f"{Style.BRIGHT}{Fore.GREEN}{message}{Style.RESET_ALL}")
        case 'STEP':
            node.get_logger().info(f"{Fore.MAGENTA}{message}{Style.RESET_ALL}")
        case 'NEW_OBSTACLE':
            node.get_logger().info(f"{Style.BRIGHT}{Fore.BLUE}{message}{Style.RESET_ALL}")
        case 'WARN':
            node.get_logger().warn(f"{Fore.YELLOW}{message}{Style.RESET_ALL}")
        case 'BIG_WARN':
            node.get_logger().warn(
                f"{Style.BRIGHT}{Fore.YELLOW}{message}{Style.RESET_ALL}")
        case 'ERROR':
            node.get_logger().error(f"{Fore.RED}{message}{Style.RESET_ALL}")
        case 'CRITICAL_ERROR':
            node.get_logger().error(
                f"{Style.BRIGHT}{Fore.RED}{message}{Style.RESET_ALL}")
        case _:
            node.get_logger().info(message)
            return 1
    return 0


class StateMachine:
    def __init__(self, states, log_node):
        self.states = states
        self.current_state = states[0]
        self.name_to_index = {name: i for i, name in enumerate(states)}
        self.log_node = log_node
        #self.curr_undex

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
            log(self.log_node, f"You are trying to set incorrect state: {state}", 'CRITICAL_ERROR')
        index = self.name_to_index[state]
        if self.current_state != state:
            log(self.log_node, f'Transit from {self.current_state} to {state}', 'NEW_OBSTACLE')
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

        self.fully_initialized = False

        # Создаем машину состояний состояющую из препятствий
        self.state_machine = StateMachine(
                ['just_follow', 'traffic_light', 'T_crossroad', 'works_sign', 'parking_sign', 'crossing_sign', 'tunnel_sign'],
                log_node=self
            )
        if state is not None:
            self.state_machine.set_state(state)

        self.obstacles = {
            'tunnel': TunnelObstacle(),
            'parking': ParkingObstacle()
        }

        self.detector = Detector() # YOLOv11 detector of signs
        self.lane_follow = LaneFollowing() # LaneFollower class
        self.not_finished = True
        self.mode = None

        # additional variables for moving
        self.boxes = None
        self.error = None

        # information about robot position and speed
        self.position = None
        self.orientation = None

        self.can_move = True #!!!!!!!!!!!! ПОМЕНЯТЬ НА False
        self.side = None
        self.linear_velocity = None
        self.angular_velocity = None

        # information about spawn position
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_z', 0.0)
        self.declare_parameter('spawn_angle', 0.0)  # Значение по умолчанию

        self.spawn_odom = {
            'pos': (self.get_parameter('spawn_x').value, self.get_parameter('spawn_y').value),
            'orient': self.get_parameter('spawn_angle').value,
            'linear_v': 0,
            'angular_v': 0
        }

        self.spawn_x = self.get_parameter('spawn_x').value
        self.spawn_y = self.get_parameter('spawn_y').value
        self.spawn_z = self.get_parameter('spawn_z').value
        self.spawn_angle = self.get_parameter('spawn_angle').value

        self.get_logger().info(f"\n\
                               Начальный X: {self.spawn_x}\n\
                               Начальный Y: {self.spawn_y}\n\
                               Начальный Z: {self.spawn_z}\n\
                               Начальный угол поворота: {self.spawn_angle}\n")

        # everything from topics
        self.frame = None
        self.cv_image = None
        self.depth_image = None
        self.lidar_scan = None
        self.yolo_result = None
        self.yolo_image = None

        #ДЛЯ ПЕРЕКРЁСТКА
        self.stop_flag = False
        self.ped_can_move_flag = False

        self.bridge = CvBridge()

        # subscribers functions
        # Переменные для выполнения заданий движения
        self.current_task = None
        self.target_odom = None

        self.create_subscription(
            Image,
            '/color/image',
            self._camera_callback,
            10)
        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10)
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
    """
    This block is for sensors callback
    Just look to next block for aviable functions
    """

    def _lidar_callback(self, msg):
        try:
            self.lidar_scan = msg
        except Exception as e:
            self.get_logger().error(f"Failed to convert lidar cloud: {e}")

    def _depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def get_bbox_depth_info(self, bbox, min_distance_partition=0.25, method='mean', show_box=False):
        depth_image = self.get_depth()

        y_min = int(bbox['y_min'])
        y_max = int(bbox['y_max'])
        x_min = int(bbox['x_min'])
        x_max = int(bbox['x_max'])
        depth_hummer = depth_image[y_min:y_max, x_min:x_max]

        sorted_values = np.sort(depth_hummer.flatten())
        min_part = sorted_values[:int(len(sorted_values) * min_distance_partition)]
        mean_value = np.mean(min_part)

        # if show_box:
            # cv2.imshow('Depth hummer', depth_hummer)
            # cv2.waitKey(1)
            # log(self, f"Mean for {min_distance_partition} min pixels: {mean_value}", 'INFO')

        return mean_value

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

    def get_sectored_lidar(self):
        """ Разделение лидара на сектора, сейчас 18 секторов по 20 градусов.
            Сектора оцентрованы (нулевой сектор включает в себя углы от -10 до +10 от направления робота).
            Для каждого сектора выводится одно (минимальное) значение.
            Углы идут против часовой стрелки, то есть прямо - 0, лево ~ 5, зад 9, право 13, кстати косяк надо чтобы делилось на 4:(

            #TODO возможно в будущем сделать настройку чтобы задвать размер сектора в аргументе

        Returns:
            list[float]: Список из минимальных расстояний для каждого из 18 секторов.
        """
        try:
            distances = self.lidar_scan.ranges
            normalized_distances = np.nan_to_num(
                distances, nan=0, posinf=10, neginf=0)

            sector_step = 20
            window_radius = 10
            # Рассчитываем усредненные значения для каждого сектора
            sectored_distances = []
            for center_index in range(0, 360, sector_step):
                # Определяем границы окна
                start_index = (center_index - window_radius) % 360
                end_index = (center_index + window_radius + 1) % 360
                # Собираем подмассив
                if start_index < end_index:
                    window = normalized_distances[start_index:end_index]
                else:
                    window = np.concatenate(
                        (normalized_distances[start_index:], normalized_distances[:end_index]))
                # Берем среднее из трех наименьших значений
                # mean_of_smallest = np.mean(np.partition(window, 2)[:3])
                # sectored_distances.append(mean_of_smallest)
                # Берем наименьшее
                sectored_distances.append(min(window))
            return sectored_distances
        except Exception as e:
            log(self, F"Error in robot.get_sectored_lidar: {e}", 'ERROR')

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

    def get_rotate_angle(self):
        x, y, z, w = self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) + self.spawn_angle
        return yaw

    def get_normalized_odometry(self, use_start_odom=False) -> dict:
        """Берет и обрабатывает значения из топика одометрии приводя к нормальному виду.

        Returns:
            dict: {
                pos: (x, y) точка где робот находится относительно момента спавна,
                orient: z градус в радианах от момента спавна
                linear_v: текущая скорость по прямой (не проверял насколько правильно работает)
                angular_v: текущая скорость вращения (не проверял насколько правильно работает)
            }
        """
        try:
            odom = self.get_odometry()

            # Переводим блядские кватернионы в радианы
            def quaternion_to_z_angle(q):
                # q is a quaternion [x, y, z, w]
                w = q[3]
                theta = 2 * np.arccos(w)
                return theta
            q = [
                    odom['orientation'].x,
                    odom['orientation'].y,
                    odom['orientation'].z,
                    odom['orientation'].w
            ]
            z = quaternion_to_z_angle(q)

            if use_start_odom is True:
                real_x = odom['position'].x - self.spawn_odom['pos'][0]
                real_y = odom['position'].y - self.spawn_odom['pos'][1]
                real_orient = z - self.spawn_odom['orient']

                normalized_odom = {
                    'pos': (real_x, real_y),
                    'orient': real_orient,
                    'linear_v': odom['linear_velocity'].x,
                    'angular_v': odom['angular_velocity'].z
                }
            else:
                normalized_odom = {
                    'pos': (odom['position'].x, odom['position'].y),
                    'orient': z,
                    'linear_v': odom['linear_velocity'].x,
                    'angular_v': odom['angular_velocity'].z
                }
            return normalized_odom
        except Exception as e:
            log(self, f"Error in robot.get_normalized_odometry: {e}", 'ERROR')

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



    # traffic_lights moving
    def obey_traffic_lights(self):
        # additional function
        def get_traffic_box(boxes):
            if boxes is not None:
                if len(boxes) > 1:
                    boxes = sorted(self.boxes, key=lambda x: x['area'])
                for box in boxes:
                    if box['label'].startswith('traffic'):
                        return box['label']
            return 'None'

        traffic_light_color = get_traffic_box(self.boxes)

        if traffic_light_color.endswith('green') and not self.can_move:
            self.can_move = True
            self.get_logger().info(f"Traffic light: {traffic_light_color}. Game started!")

        if traffic_light_color.endswith('green') or (traffic_light_color == 'None' and self.can_move):
            #self.lane_follow.just_follow(self)
            self.state_machine.set_state('just_follow')

    # T-cross completing function
    def T_cross_road(self):
        # additional function
        def get_cross_road(boxes):
            if self.boxes is not None:
                boxes = self.boxes
                if len(boxes) > 1:
                    boxes = sorted(self.boxes, key=lambda x: -x['area'])
                for box in boxes:
                    if box['area'] > 13000:
                        if box['label'].startswith('right') or box['label'].startswith('left'):
                            return box['label']
            return None

        side = get_cross_road(self.boxes)
        angular_z = 1
        cross_speed = 0.165
        if side is not None:
            side = side.split('_')[0]
            if self.side is None:
                self.get_logger().info(f"Current side found for T-crossing: {side}")
                self.side = side
            if side == 'left':
                angular_z = 0.8

        if side is None:
            if self.side == 'right':
                angular_z = 0.8
        self.lane_follow.just_follow(self, speed=cross_speed, z_speed=angular_z, hold_side=self.side)

    def pedestrian_crossing(self):
        #cv2.imshow('pedastrial',self.depth_image[200:220, 230:670])
        #self.lane_follow.just_follow(self, speed=0.1)
        #self.get_logger().info(f'\n\nPedastrial!!!')
        if not self.stop_flag:
            self.stop_flag = True
            self.lane_follow.just_follow(self)
            #self.get_logger().info(f'MOVE')
            for box in self.boxes:
                if box['label'] == 'crossing_sign' and box['conf'] > 0.50:
                    self.stop_flag = False
            return

        #self.get_logger().info(f'STOP')
        if not self.ped_can_move_flag:
            self.lane_follow.stop(self)
            #x, y, z, w = self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w
            #yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) + self.spawn_angle
            yaw = self.get_rotate_angle()
            ang_speed = 3.14 / 32
            if yaw >= 0:
                ang_speed = -ang_speed
            #if yaw <= 0 else -(3.14 / 32)

            self.get_logger().info(f'rad: {yaw}, deg: {np.degrees(yaw)}')

            if np.degrees(yaw) >= 1 or np.degrees(yaw) <= -1: #or np.degrees(yaw) >= 1 or np.degrees(yaw) <= -1 or np.degrees(yaw) >= 89 or np.degrees(yaw) <= -91 or np.degrees(yaw) >= -89 or np.degrees(yaw) <= -91: # 1, -1
                self.move(linear_x=0.0, angular_z=ang_speed)
            else:
                self.move(linear_x=0.0, angular_z=0.0)
                self.ped_can_move_flag = True

        if self.ped_can_move_flag:
            #cv2.imshow('pedastrial',self.depth_image[150:250, 180:670])
            #self.lane_follow.just_follow(self, 0.0, 0.0)
            self.get_logger().info(f'{np.min(self.depth_image[200:220, 230:670])}')

            if np.min(self.depth_image[200:220, 230:670]) > 0.3 and np.min(self.depth_image[200:220, 230:670]) != float('-inf'):
                self.move_task(0.35, 0.5)
                #self.rotate_task(-np.pi/2)
                self.lane_follow.start(self)
                self.state_machine.set_state('just_follow')

    def move_task(self, distance, linear_x=0.15):
        """Дает задание для движения прямо ровно на заданную дистанцию.

        Args:
            distance (_type_): Дистанция сколько должен проехать в текущем направлении
            linear_x (float, optional): Начальная скорость (ни на что не влияет, #TODO:убрать). Defaults to 0.15.

        Returns:
            _type_: Просто ноль, #TODO:Убрать.
        """
        odom = self.get_normalized_odometry()
        new_x = odom['pos'][0] + np.cos(odom['orient']) * distance
        new_y = odom['pos'][1] + np.sin(odom['orient']) * distance
        self.target_odom = {
            'pos': (new_x, new_y),
            'orient': odom['orient'],
            'linear_v': linear_x,
            'angular_v': odom['angular_v']
        }
        log(self, f"Registered move task for {distance} distance", 'INFO')
        self.move(linear_x=linear_x)
        self.current_task = 'move'
        log(self, f"Cur odom: {odom}", 'INFO')
        log(self, f"Target odom: {self.target_odom}", 'INFO')
        return 0

    def rotate_task(self, angle_diff=None, fixed_angle=None, angular_v=np.pi / 2):
        """Дает задание для поворота ровно на заданное количество радиан.

        Args:
            angle (_type_): Радианы для поворота
            angular_v (_type_, optional): Начальная скорость (ни на что не влияет, #TODO:убрать). Defaults to np.pi/2.

        Returns:
            _type_: Просто ноль, #TODO:Убрать.
        """
        odom = self.get_normalized_odometry()
        if angle_diff is not None:
            new_orient = odom['orient'] + angle_diff
        elif fixed_angle is not None:
            new_orient = fixed_angle
        else:
            return 0
        if new_orient < 0:
            new_orient = np.pi + (np.pi + new_orient)
        log(self, f"Old orient: {odom['orient']}, new: {new_orient}", 'INFO')
        self.target_odom = {
            'pos': odom['pos'],
            'orient': new_orient,
            'linear_v': odom['linear_v'],
            'angular_v': angular_v
        }
        log(self, f"Registered rotate task for {angle_diff} distance", 'INFO')
        self.move(angular_z=angle_diff)
        self.current_task = 'rotate'
        log(self, f"Cur odom: {odom}", 'INFO')
        log(self, f"Target odom: {self.target_odom}", 'INFO')
        return 0

    def is_task_completed(self, epsilon=0.02, min_v=0.025, max_v=0.75, min_w=np.pi / 8, max_w=np.pi / 3, safe_mode=True):
        """Проверяет выполнено ли задание, возвращает ответ, регулирует скорость.
            Должна первоочередно вызываться в robot.process_mode() для правильной обработки заданий.

        Args:
            epsilon (float, optional): Точность для линейных заданий, epsilon / 8 для вращательных заданий. Defaults to 0.01.
            min_v (float, optional): #TODO:убрать. Defaults to 0.025.
            max_v (float, optional): Стандартная линейная скорость, которая умножается на величину ошибки между целевыми координатами и текущими. Defaults to 1.25.
            min_w (_type_, optional): #TODO:убрать.. Defaults to np.pi/8.
            max_w (_type_, optional): Стандартная скорость вращения, которая умножается на величину ошибки между целевым радианом и текущим. Defaults to np.pi/3.

        Returns:
            _type_: True если задание выполнены, иначе False
        """
        odom = self.get_normalized_odometry()

        err_x = max(self.target_odom['pos'][0], odom['pos'][0]) - min(self.target_odom['pos'][0], odom['pos'][0])
        err_y = max(self.target_odom['pos'][1], odom['pos'][1]) - min(self.target_odom['pos'][1], odom['pos'][1])
        err_a = self.target_odom['orient'] - odom['orient']
        # log(self, f"Cur odom: {odom}", 'INFO')
        # log(self, f"Error x: {err_x:.3f}, error y: {err_y:.3f}, error angle: {err_a:.4f}", 'WARN')
        if safe_mode is True:
            distances = self.get_sectored_lidar()

        if self.current_task == 'move':
            if safe_mode:
                if min(distances[-1], distances[0], distances[1]) < 0.2:
                    self.current_task = None
                    self.target_odom = None
                    self.move(0.0, 0.0)
                    log(self, "!UNSAFE DISTANCE! Finished move task", 'ERROR')
                    log(self, f"Distances were: {[distances[-1], distances[0], distances[1]]}")
                    time.sleep(0.1)
                    return True
            if abs(err_x) < epsilon and abs(err_y) < epsilon:
                if random.randint(0, 25) == 1:
                    log(self, f"Cur pos {odom['pos']}", 'INFO')
                self.current_task = None
                self.target_odom = None
                log(self, "Finished move task", 'INFO')
                self.move(0.0, 0.0)
                time.sleep(0.1)
                return True
            new_v = max(err_x, err_y) * max_v
            self.move(linear_x=new_v)
            return False
        elif self.current_task == 'rotate':
            # if random.randint(0, 100) == 1:
                # log(self, f"Cur orient {odom['orient']}", 'INFO')
            if abs(err_a) < epsilon / 10:
                self.current_task = None
                self.target_odom = None
                log(self, f"Finished rotate task", 'INFO')
                self.move(0.0, 0.0)
                time.sleep(0.1)
                return True
            new_w = err_a * max_w
            if new_w > max_w:
                new_w = max_w
            elif new_w < -max_w:
                new_w = -max_w
            # if random.randint(0, 100) == 1:
                # log(self, f"Cur w {new_w}", 'INFO')
            self.move(angular_z=new_w)
            return False
        return False

    # Main function

    def process_mode(self):
        try:
            # Проверка загрузились ли все датчики
            if self.fully_initialized is False:
                if (
                    self.get_lidar() is not None and
                    self.get_depth() is not None and
                    self.get_image() is not None and
                    self.get_odometry() is not None and
                    self.yolo_image is not None
                ):
                    time.sleep(0.1)   #TODO: убрать, но пока пусть спит перед стартом тормоз ебаный
                    log(self,
                        f"Robot interface initialized with state {self.state_machine.get_state()}", "GOOD_INFO")
                    self.fully_initialized = True
                else:
                    return None

            # Проверка выполнения задания
            if self.current_task is not None:
                if self.state_machine.get_state() == 'parking_sign':
                    if not self.is_task_completed(epsilon=0.025):
                        return None
                elif self.state_machine.get_state() == 'just_follow':
                    if not self.is_task_completed(epsilon=0.1):
                        return None
                elif not self.is_task_completed():
                    return None

            elif self.cv_image is not None:
                # self.lane_follow.just_follow(self)
            # if random.randint(0, 10) == 1:
                # self.get_logger().info(f"Lane was processed for {time.time() - start_time}s")

                self.mode = self.state_machine.get_state()
                #self.get_logger().info(f'{self.mode}')

                #['traffic_light', 'T_crossroad', 'works_sign', 'parking_sign', 'crossing_sign', 'tunnel_sign', 'just_follow']
                match (self.mode):
                    case 'traffic_light':
                        self.obey_traffic_lights()
                    case 'T_crossroad':
                        self.T_cross_road()
                    case 'works_sign':
                        self.state_machine.set_state('just_follow') # Заглушка, т.к. состояние не ресетается после выполнения перекрёстка
                        # Функция прохождения лабиринта
                    case 'parking_sign':
                        # self.move_task(0.3)
                        self.state_machine.set_state('parking_sign') # Заглушка
                        self.obstacles['parking'].process(self)
                    case 'crossing_sign':
                        self.lane_follow.just_follow(self, speed=0.0)
                        self.pedestrian_crossing()
                    case 'tunnel_sign':
                        self.obstacles['tunnel'].process(self)
                    case 'just_follow':
                        #TODO:тут правая сторона для нормального выезда с туннеля, после приближения к светофору нужно будет обязательно переключиться на двойную линию.
                       if self.can_move:
                           self.lane_follow.just_follow(self, hold_side='right')
                    case _:
                        self.lane_follow.just_follow(self)
        except Exception as e:
            log(self, f"Robot.process_mode(): {e}", "CRITICAL_ERROR")

    # Запуск детектора yolo
    def run_detector(self) -> None:
        # Запускает детектор, который получает боксы из YOLO в виде списка словарей, посмотреть структуру можно внутри функции
        try:
            if self.cv_image is not None:
                # TODO: сделать блокировку
                self.boxes, self.yolo_image, _ = self.detector.process_image(self.cv_image)
                self.check_for_state_transition(self.boxes)
                cv2.imshow('YOLO', self.yolo_image)
                cv2.waitKey(1)
        except Exception as e:
            pass
            #self.get_logger().error(
                #f"Robot.run_detector exception: {Fore.RED}{e}{Style.RESET_ALL}")

    # Определение испытания по bb из yolo
    def check_for_state_transition(self, boxes):
        # Возмонжо стоит поменять, запускается из детектора и пытается перейти к следующему состоянию по условию
        curr_depth_image = self.depth_image
        for box in boxes:
            #self.get_logger().info(f"{box['x_min']}, {box['y_min']}")
            #self.get_logger().info(f"{box['x_max']}, {box['y_max']}")
            #break
            bbox = curr_depth_image[int(box['y_min']):int(box['y_max']), int(box['x_min']):int(box['x_max'])]
            #self.get_logger().info(f'{bbox[len(bbox)//2, len(bbox[0])//2]}')

            if not self.can_move and box['label'].startswith('traffic') and box['conf'] > 0.80: #светофор
                self.get_logger().info(f"{box['label']}: {box['conf']} conf")
                self.state_machine.set_state('traffic_light')


            if bbox[len(bbox)//2, len(bbox[0])//2] > 0.3: # скип если далеко
                continue

            elif box['label'] == 'T_crossroad' and box['conf'] > 0.85: #Развилка
                self.get_logger().info(f"{box['label']}: {box['conf']} conf")
                self.state_machine.set_state('T_crossroad')

            elif box['label'] == 'works_sign' and box['conf'] > 0.85: #Стены
                self.get_logger().info(f"{box['label']}: {box['conf']} conf")
                self.state_machine.set_state('works_sign')

            elif box['label'] == 'parking_sign' and box['conf'] > 0.85: #Парковка
                self.get_logger().info(f"{box['label']}: {box['conf']} conf")
                self.state_machine.set_state('parking_sign')

            elif box['label'] == 'crossing_sign' and box['conf'] > 0.85: #Пешеходный переход
                self.get_logger().info(f"{box['label']}: {box['conf']} conf")
                self.state_machine.set_state('crossing_sign')

            elif box['label'] == 'tunnel_sign' and box['conf'] > 0.85: #Тунель
                self.get_logger().info(f"{box['label']}: {box['conf']} conf")
                self.state_machine.set_state('tunnel_sign', )
                # self.get_logger().info(f"Now state is: {self.state_machine.get_state()} conf")

            else:
                pass


# Class for lane following code
class LaneFollowing():
    def __init__(self, h=None, w=None):
        h_real = h//4 if h else 212
        w_real = w//6 if w else 60

        # constant variables for giving an good moving in cross road
        self.const1, self.const2 = (h_real, w_real+15), (h_real*3, w_real-15)
        # previous points
        self.prevpt1, self.prevpt2 = (h_real, w_real), (h_real*3, w_real)
        self.is_stop = False

    def stop(self, robot):
        robot.move(linear_x=0.0, angular_z=0.0)
        self.is_stop = True

    def start(self, robot):
        self.is_stop = False

    def just_follow(self, robot: Robot, speed=0.1, z_speed=0.5, hold_side=None):
        if not self.is_stop:
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
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))

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
                if hold_side == 'right':
                    cpt1 = self.const1
                cpt2 = tuple(centroids[min_idx2]
                            ) if min_dist2 < 100 else self.prevpt2
                if hold_side == 'left':
                    cpt2 = self.const2
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
            robot.move(linear_x=speed, angular_z=(
                self.error * 90.0 * z_speed / 400) / 15)
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
                cv2.imshow('Lane camera', concatenated_image)
                cv2.waitKey(1)


# Класс-родитель препятствие в каждом из которых в process будет реализована логика прохождения препятствия
class Obstacle:
    def __init__(self):
        return None

    def process(self, robot: Robot):
        return None

# Класс препятствия для туннеля


class TunnelObstacle(Obstacle):
    def __init__(self):
        super().__init__()
        self.min_wall_distance = 0.175
        self.min_obstacle_distance = 0.2
        self.max_speed = 0.15
        self.max_rad_speed = 1.07

        self.min_rad = None
        self.max_rad = None
        self.angle_diff = None
        self.distance_diff = None

        self.state = 0

    def filter_lidar_angles(self, distances, odom, robot=None):
        """Берет дистанции с лидара, и задает угол для поворота по направлению к самому дальнему месту, также задает половину от расстояния к этому месту.
            Логика такая, что у нас на шаге 0 алгоритма были зафикисрованы углы правой и левой стены (между ними 90 градусов) и
            мы максимальные расстояния с лидаров будем брать только с тех секторов лидара, которые не выходят из этого разрешенного диапазона радиан.
            Таким образом робот все время будет двигаться туда, где максимальная дистанция, засчет того что он никогда не развернется назад,
            он точно прибудет к выходу туннеля.
            Также стоит отметить что сектора у которых расстояние 10 не берутся, так как мы при предобработке лидара posinf превращаем в 10,
            => мы не можем ехать в те направления так как это пустота (дырка в туннеле передает привет от Кудинова)

        Args:
            distances (_type_): Сектора из лидара.
            odom (_type_): Нормализованная одометрия.
            robot (_type_, optional): объект робота для логгиррвания. Defaults to None.
        """
        sector_step = 20
        sectors = 18
        max_i = -1
        max_dst = -1

        orient = odom['orient']
        left_range = self.max_rad - orient      # на сколько радиан слева можем повернуть
        right_range = orient - self.min_rad     # на сколько радиан справа можем повернуть

        max_left_index = math.ceil(np.degrees(left_range) / sector_step)     # сколько индексов слева можем рассматривать
        max_right_index = math.ceil(np.degrees(right_range) / sector_step)   # сколько индексов слева можем рассматривать
        for i, distance in enumerate(distances):
            if i >= max_left_index and i <= (sectors - max_right_index):
                continue
            if distance > max_dst and distance < 10:
                max_dst = distance
                max_i = i
        if max_i > (sectors / 2):
            self.angle_diff = -np.radians((sectors - max_i) * sector_step)
        else:
            self.angle_diff = np.radians(max_i * sector_step)
        # log(robot, f"Sectors: {sectors}, max_i: {max_i}, sector_step: {sector_step}", 'WARN')
        self.distance_diff = max_dst * 0.6
        log(robot, f"-----------------------------------", 'GOOD_INFO')
        log(robot, f"Cur odom: {odom}", 'GOOD_INFO')
        log(robot, f"Angle diff: {self.angle_diff}", 'GOOD_INFO')
        log(robot, f"Distance diff: {self.distance_diff}", 'GOOD_INFO')
        log(robot, f"l_range: {left_range}, r_range: {right_range}, max_left_idx: {max_left_index}, max_right_idx: {max_right_index}, max_idx: {max_i}", 'GOOD_INFO')

    def process(self, robot: Robot):
        try:
            sectored_distances = robot.get_sectored_lidar()
            odom = robot.get_normalized_odometry()

            # log(robot, f"ODOM: {odom}", 'INFO')
            match self.state:
                case 0:
                    log(robot, "Starting process TUNNEL", 'NEW_OBSTACLE')
                    # Фиксируем углы крайней правой и крайней левой стены относительно робота.
                    self.min_rad = odom['orient']
                    self.max_rad = odom['orient'] + np.pi / 2
                    # log(robot, f"Min rad: {self.min_rad} Max rad: {self.max_rad} Cur rad: {odom['orient']}", 'GOOD_INFO')
                    self.state += 1
                    robot.move_task(0.4)
                case 1:
                    # Повернемся по направлению где наибольшая дистанция
                    for box in robot.boxes:
                        if box['label'] == 'traffic_light_green' and box['conf'] > 0.6:
                            log(robot, "Leaving tunnel")
                            robot.lane_follow.just_follow(robot, hold_side='right')
                            robot.state_machine.set_state('just_follow')
                            return None
                    self.filter_lidar_angles(sectored_distances, odom, robot)
                    robot.rotate_task(self.angle_diff)
                    self.state += 1
                case 2:
                    # Проедем половину наибольшей дистанции и вернемся к предыдущему шагу
                    for box in robot.boxes:
                        if box['label'] == 'traffic_light_green' and box['conf'] > 0.6:
                            log(robot, "Leaving tunnel")
                            robot.lane_follow.just_follow(robot, hold_side='right')
                            robot.state_machine.set_state('just_follow')
                            return None
                    robot.move_task(self.distance_diff)
                    self.state -= 1
                #     log(robot, f"FINISH Error x: {err_x:.3f}, error y: {err_y:.3f}, error angle: {err_a:.4f}", 'GOOD_INFO')
            return None

        except Exception as e:
            log(robot, f"Fail in TunnelObstacle.process: {e}", 'ERROR')


class ParkingObstacle(Obstacle):
    def __init__(self):
        super().__init__()

        self.state = 0
        self.is_hummer_left = None

    def process(self, robot: Robot):
        try:
            sectored_distances = robot.get_sectored_lidar()
            odom = robot.get_normalized_odometry()

            # log(robot, f"ODOM: {odom}", 'INFO')
            match self.state:
                case 0:
                    log(robot, "Starting process PARKING", 'NEW_OBSTACLE')
                    # robot.move_task(0.475)
                    self.state += 1
                case 1:
                    # robot.rotate_task(np.pi / 2)
                    # self.state += 1
                    robot.lane_follow.just_follow(robot, hold_side='left')
                    for box in robot.boxes:
                        if box['label'] == 'hummer_front':
                            hummer_depth = robot.get_bbox_depth_info(box)
                            log(robot, f"Hummer depthj: {hummer_depth}")
                            if hummer_depth < 0.45:
                                # image_middle = robot.cv_image.shape[1] // 2
                                # hummer_center_x = (box['x_min'] + box['x_max']) // 2
                                # if hummer_center_x >= image_middle:
                                #     self.is_hummer_left = False
                                # else:
                                #     self.is_hummer_left = True
                                robot.move(0.0, 0.0)
                                self.state += 1
                                log(robot, "Swithing for double line control", 'STEP')
                case 2:
                    robot.lane_follow.just_follow(robot)
                    # Работает для настройки на 18 секторов
                    # 15,14,13: (1.8071762, 1.2126777, 10.0)
                    # log(robot, f"Sectors 2,3,4,5: {sectored_distances[2], sectored_distances[3], sectored_distances[4], sectored_distances[5]}")
                    if sectored_distances[3] < .225 and sectored_distances[4] < .225 and sectored_distances[5] < .225:
                        robot.move(0.0)
                        self.state += 1
                        self.is_hummer_left = True
                        # robot.move_task(0.1)
                        log(robot, "Turning to left parking space", 'STEP')
                    # log(robot, f"Sectors 16,15,14,13: {sectored_distances[16], sectored_distances[15], sectored_distances[14], sectored_distances[13]}")
                    if sectored_distances[15] < .225 and sectored_distances[14] < .225 and sectored_distances[13] < .225:
                        robot.move(0.0)
                        self.state += 2
                        self.is_hummer_left = False
                    #     # robot.move_task(0.1)
                        log(robot, "Turning to right parking space", 'STEP')
                case 3:
                    robot.rotate_task(-np.pi / 4)
                    self.is_hummer_left = True
                    self.state += 2
                case 4:
                    robot.rotate_task(np.pi / 4)
                    self.is_hummer_left = False
                    self.state += 1
                case 5:
                    robot.move_task(0.4)
                    self.state += 1
                case 6:
                    log(robot, "Turning back", 'STEP')
                    robot.rotate_task(np.pi)
                    self.state += 1
                case 7:
                    robot.move_task(0.4)
                    self.state += 1
                case 8:
                    if self.is_hummer_left is True:
                        robot.rotate_task(np.pi / 4)
                    else:
                        robot.rotate_task(-np.pi / 4)
                    self.state += 1
                case 9:
                    robot.move_task(0.45)
                    self.state += 1
                case 10:
                    robot.lane_follow.just_follow(robot, hold_side='left')
                    # self.state += 1
                # case 10:
                #     for box in robot.boxes:
                #         if box['label'] == 'parking_sign':
                #             return 0
                #     robot.state_machine.set_state('just_follow')
                #log(robot, f"FINISH Error x: {err_x:.3f}, error y: {err_y:.3f}, error angle: {err_a:.4f}", 'GOOD_INFO')
            return None

        except Exception as e:
            log(robot, f"Fail in ParkingObstacle.process: {e}", 'ERROR')
