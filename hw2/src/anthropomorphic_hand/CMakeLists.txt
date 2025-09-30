cmake_minimum_required(VERSION 3.8)
project(anthropomorphic_hand)

find_package(ament_cmake REQUIRED)

# Установка launch, urdf, rviz файлов
install(DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

# Установка Python-скриптов
install(PROGRAMS
  scripts/arm_wave.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
