import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
def generate_launch_description():
    # Get the urdf file
    TURTLEBOT3_MODEL = 'waffle_pi'
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('seeker_swarm'),
        'models',
        model_folder,
        'model.sdf'
    )

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    count=10
    x_val_pose = [-3.0,-10.0,-10.0,7.0,10.0,5.0,-5.0,10.0,30.0,5.0]
    y_val_pose = [5.0,-1.0,-5.0,2.0,3.0,-1.0,-15.0,-20.0,0.0,-10.0]
    for i in range(count):
        robot_name = "robot_" + str(i)
        x_val = str(float(x_val_pose[i]))
        y_val = str(float(y_val_pose[i]))
        node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot_name,
                '-file', urdf_path,
                '-x', x_val,
                '-y', y_val,  # Adjust the Y position as needed
                '-z', '0.01',
                '-robot_namespace', robot_name
            ],
            output='screen',
        )
        ld.add_action(TimerAction(period=0.1 + float(i * 2), actions=[node],))
    # Add any conditioned actions
    # ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld