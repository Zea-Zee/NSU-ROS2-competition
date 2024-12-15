#!/bin/bash

# Очистка консоли
clear

ORIGINAL_DIR=$(pwd)
WORKSPACE="$HOME/ros2_ws"
PACKAGES_DIR="$ORIGINAL_DIR/ros2_packages"

# Функция для обработки завершения скрипта
cleanup() {
    echo "Завершение работы..."
    # Возврат в исходную директорию
    cd "$ORIGINAL_DIR"
    echo "Возврат в директорию: $ORIGINAL_DIR"
}

# Устанавливаем обработчик сигнала SIGINT
trap cleanup SIGINT

# Убедимся, что рабочее пространство существует
echo "Рабочее пространство: $WORKSPACE"
if [ ! -d "$WORKSPACE/src/" ]; then
    echo "Рабочее пространство $WORKSPACE не инициализировано!"
    exit 1
fi

# Очистка сборки
echo "Очистка сборки..."
cd $WORKSPACE
rm -rf build log install src/*
colcon build
source install/setup.bash

# Копируем пакеты в папку src рабочего пространства
echo "Копирование пакетов..."
cp -r $PACKAGES_DIR/my_robot_controller $PACKAGES_DIR/referee_console $PACKAGES_DIR/robot_bringup $PACKAGES_DIR/robot_description $WORKSPACE/src/

# Выполняем сборку
echo "Сборка ROS 2 пакетов..."
colcon build

# Очистка консоли
#clear

# Запуск Gazebo и ROS 2 с переданными аргументами
echo "Запуск Gazebo и ROS 2..."
ros2 launch robot_bringup autorace_2023.launch.py

# Возврат в исходную директорию (при завершении ros2 launch)
cleanup

