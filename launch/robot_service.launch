# launch module includes elements to launch all types of processes and actions
from launch import LaunchDescription

# launch_ros module includes elements to launch ROS 2 processes and actions
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tcp_server',
            executable='robot_service',
        )
    ])