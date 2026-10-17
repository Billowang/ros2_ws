from launch import LaunchDescription
from launch_ros.actions import Node

NODES = ['web_ui_node', 'map_node', 'control_node', 'motor_node',
         'fusion_node', 'mpu_node', 'spray_node', 'trigger_node']


def generate_launch_description():
    return LaunchDescription([
        Node(package='field_robot', executable=n, name=n, output='screen') for n in NODES
    ])
