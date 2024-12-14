import cv2
import rclpy
import time

from geometry_msgs.msg import Twist
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from .Detector import Detector
# import threading


class Robot(Node):
    def __init__(self, camera=None, odom=None, twist=None):
        super().__init__("moving_robot")

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
## TODO: сделать блокировку
        self.cv_image = None
        self.depth_image = None
        self.lidar_scan = None
        self.yolo_processed_image = None

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

        self.create_timer(0.01, self.process_mode)
        self.create_timer(0.5, self.run_detector)

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
        if self.cv_image is not None:
            self.lane_follow.just_follow(self)

    def run_detector(self):
        """Фоновый поток для запуска YOLO-детектора."""
        if self.cv_image is not None:
## TODO: сделать блокировку
            self.yolo_process_result = self.detector.process_image(self.cv_image)

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

        # Раскоментить, чтобы видеть точки и прочее
        cv2.imshow('Camera', image)
        cv2.imshow('Distance image', dst)
        cv2.waitKey(1)
