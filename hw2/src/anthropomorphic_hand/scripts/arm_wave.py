#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class ArmAnimator(Node):
    def __init__(self):
        super().__init__('arm_animator')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Гц

        self.joint_names = [
            "shoulder_joint_x",
            "shoulder_joint_y",
            "shoulder_joint_z",
            "elbow_joint",
            "wrist_joint_x",
            "wrist_joint_y",
            "wrist_joint_z",
        ]

    # ====== Движения ======
    def move_wave(self, t):
        """Махаем запястьем по оси Y, ладонь уже повернута"""
        shoulder = 1.0
        elbow = -shoulder
        wrist_y = 0.6 * math.sin(7.0 * t)
        wrist_z = math.pi / 2
        return shoulder, elbow, wrist_y, wrist_z

    def move_raise(self, t):
        """Поднимаем плечо и плавно поворачиваем ладонь"""
        progress = min(t / 2.0, 1.0)
        shoulder = progress
        elbow = -shoulder
        wrist_y = 0.0
        wrist_z = progress * (math.pi / 2)
        return shoulder, elbow, wrist_y, wrist_z

    def move_lower(self, t):
        """Опускаем плечо и плавно возвращаем ладонь"""
        progress = min(t / 2.0, 1.0)
        shoulder = (1.0 - progress) * 1.0
        elbow = -shoulder
        wrist_y = 0.0
        wrist_z = (1.0 - progress) * (math.pi / 2)
        return shoulder, elbow, wrist_y, wrist_z


    # ====== Основной колбэк ======
    def timer_callback(self):
        t = self.get_clock().now().nanoseconds / 1e9 - self.start_time

        if t < 2.0:
            shoulder, elbow, wrist_y, wrist_z = self.move_raise(t)
        elif t < 6.0:
            shoulder, elbow, wrist_y, wrist_z = self.move_wave(t - 2.0)
        elif t < 8.0:
            shoulder, elbow, wrist_y, wrist_z = self.move_lower(t - 6.0)
        else:
            shoulder, elbow, wrist_y, wrist_z = 0.0, 0.0, 0.0, 0.0

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [
            shoulder,  # shoulder_x
            0.0,       # shoulder_y
            0.0,       # shoulder_z
            elbow,     # elbow
            0.0,       # wrist_x
            wrist_y,   # мах рукой
            wrist_z    # поворот ладони
        ]

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmAnimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
