from ultralytics import YOLO
from geometry_msgs.msg import Twist
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rclpy
import threading
import time
import cv2
import numpy as np
import os


class Detector:
    def __init__(self, model_path: str=f"{os.getcwd()}/src/my_robot_controller/model/best.pt", confidence_threshold: float=0.2):
        # Загрузка модели YOLOv11n
        self.model = YOLO(model_path)
        self.confidence_threshold = confidence_threshold
        self.processed_image = None
        self.lock = threading.Lock()  # Для потокобезопасного обновления изображения

    def process_image(self, image):
        """Запускает инференс YOLO и возвращает изображение с размеченными объектами."""
        result = self.model(image)[0]  # return a list of Results objects
        boxes = result.boxes  # Boxes object for bounding box outputs
        masks = result.masks  # Masks object for segmentation masks outputs
        keypoints = result.keypoints  # Keypoints object for pose outputs
        probs = result.probs  # Probs object for classification outputs
        obb = result.obb  # Oriented boxes object for OBB outputs

        # result.show()  # display to screen
        plot_img = result.plot()
        image_np = np.array(plot_img)
        cv2.imshow('Detector', image_np)
        cv2.waitKey(1)

        return result

    def get_processed_image(self):
        """Возвращает последнее обработанное изображение."""
        with self.lock:
            return self.processed_image
