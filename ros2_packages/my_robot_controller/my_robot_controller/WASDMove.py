import keyboard
import time
import pygame
from .Robot import Robot

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