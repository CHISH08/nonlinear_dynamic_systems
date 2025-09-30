#!/bin/bash
set -e

# Чистое окружение
unset LD_PRELOAD LD_LIBRARY_PATH PYTHONPATH

# Деактивируем conda если активна
if [ -n "$CONDA_PREFIX" ]; then
    conda deactivate
fi

# Подгружаем окружение ROS и локальное
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Запускаем launch-файл
ros2 launch anthropomorphic_hand display.launch.py
