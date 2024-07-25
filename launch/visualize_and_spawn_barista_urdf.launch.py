import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'barista_robot_model.urdf'
    #xacro_file = "box_bot.xacro"
    package_description = "barista_robot_description"
    ####### DATA INPUT END ##########

    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)

    # Robot State Publisher
    # 1. Publishes to topic /robot_description
    # 2. Publishes static tf
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'urdf_vis.rviz')


    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])


    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 0.1052]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "bar_bot"

    # Generate random name
    entity_name = robot_base_name+"-"+str(int(random.random()*100000))

    # Spawn ROBOT Set Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), 
                   '-y', str(position[1]), 
                   '-z', str(position[2]),
                   '-R', str(orientation[0]), 
                   '-P', str(orientation[1]), 
                   '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )


    # create and return launch description object
    return LaunchDescription(
        [            
            robot_state_publisher_node,
            rviz_node,
            spawn_robot
        ]
    )