from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    my_node = Node(
        package='SeekerSwarm',
        executable='robotswarm'
    )

    ld.add_action(my_node)
   

    return ld
