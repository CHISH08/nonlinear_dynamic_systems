from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('anthropomorphic_hand')
    urdf = os.path.join(pkg, 'urdf', 'anthropomorphic_arm.urdf')
    rviz_cfg = os.path.join(pkg, 'rviz', 'arm.rviz')

    return LaunchDescription([
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='jsp_gui'
        # ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rsp',
            parameters=[{'robot_description': open(urdf).read()}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_cfg],
            output='screen',
        ),
        Node(
            package='anthropomorphic_hand',
            executable='arm_wave.py',
            output='screen'
        )
    ])
