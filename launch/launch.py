import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('seeker_swarm'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(
        get_package_share_directory('seeker_swarm'),
        'worlds',
        'warehouse.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
           os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
       ),
       launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # # Get the urdf file
    # TURTLEBOT3_MODEL = 'waffle_pi'
    # model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    # urdf_path = '/opt/ros/humble/share/turtlebot3_gazebo/models/{}/model.sdf'.format(model_folder)

    # # Set the number of robots to spawn
    # count = 10  # Adjust this as needed

    ld = LaunchDescription()
    # for i in range(count):
    #     robot_name = "robot_" + str(i)
    #     x_val = str(float(i - (count / 2)))
    #     y_val = str(float(i - (count / 2)))
    #     node = Node(
    #         package='gazebo_ros',
    #         executable='spawn_entity.py',
    #         arguments=[
    #             '-entity', robot_name,
    #             '-file', urdf_path,
    #             '-x', '0.0',
    #             '-y', y_val,  # Adjust the Y position as needed
    #             '-z', '0.01',
    #             '-robot_namespace', robot_name
    #         ],
    #         output='screen',
    #     )
    #     ld.add_action(TimerAction(period=0.5 + float(i * 2), actions=[node],))


    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    
    return ld
