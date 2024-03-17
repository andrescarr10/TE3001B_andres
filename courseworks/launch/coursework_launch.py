from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    signalgenerator_node = Node(
        package='courseworks',
        executable='signal_generator',
    )

    process_node = Node(
        package='courseworks',
        executable='process'
    )

    l_d = LaunchDescription([signalgenerator_node, process_node])
    return l_d