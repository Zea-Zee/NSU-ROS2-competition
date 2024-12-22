from typing import List, Tuple
from ultralytics import YOLO
from geometry_msgs.msg import Twist
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rclpy
import time
import cv2
import numpy as np
import os

WEIGHTS_PATH = f"{os.getcwd()}/my_robot/autorace_core_roscomnadzor/model/best.pt"
#WEIGHTS_PATH = f"best.pt"my_robot/autorace_core_roscomnadzor/model/best.pt

class Detector:
    def __init__(self, model_path: str = WEIGHTS_PATH):
        self.model: YOLO = YOLO(model_path)

    def process_image(self, image: Image) -> Tuple[List[dict], np.ndarray, float]:
        """Закидывает картинку в yolo
        Returns:
            Tuple[
                List[dict] - список словарей, где каждый словарь - ббокс,
                np.ndarray - просто картинка с ббоксами,
                float - длительность инференса
            ]
        """
        start_time = time.time()
        yolo_result = self.model(image)[0]
        plot_img = yolo_result.plot()
        image_np = np.array(plot_img)
        boxes = []
        for box in yolo_result.boxes:
            class_id = int(box.cls)
            confidence = box.conf[0]
            label = yolo_result.names[class_id]
            box_result = {
                'x_min': box.xyxy[0][0],  # xmin coordinate
                'y_min': box.xyxy[0][1],  # ymin coordinate
                'x_max': box.xyxy[0][2],  # xmax coordinate
                'y_max': box.xyxy[0][3],  # ymax coordinate
                'width': box.xyxy[0][2] - box.xyxy[0][0],  # width of the box
                'height': box.xyxy[0][3] - box.xyxy[0][1],  # height of the box
                'area': (box.xyxy[0][2] - box.xyxy[0][0]) * (box.xyxy[0][3] - box.xyxy[0][1]),  # area of the box
                'label': label,
                'conf': confidence,
            }
            boxes.append(box_result)
        duration = time.time() - start_time
        return boxes, image_np, duration
