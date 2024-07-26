import os
import random

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_prefix, get_package_share_directory

# this is the function launch  system will look for
def generate_launch_description():

    # Gazebo environment variables
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_bar_bot_gazebo = get_package_share_directory('barista_robot_description')

    # We get the whole install dir path to avoid having to copy or  
    # softlink manually the packages, so that gazebo can find them
    description_package_name = "barista_robot_description"
    install_dir = get_package_prefix(description_package_name)

    # Set the path to the WORLD model files to find 
    # the models inside box_bot_gazebo package/models
    gazebo_models_path = os.path.join(pkg_bar_bot_gazebo, 'models')

    # Include local package directories to model and plugin paths
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'    

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # Declare a launch argument for Gazebo world file
    world_file_arg = DeclareLaunchArgument(
        "world",
        # default_value=os.path.join(pkg_bar_bot_gazebo, 'worlds', 'playpen.world'),
        # default_value=[os.path.join(pkg_bar_bot_gazebo, 'worlds', 'playpen.world'), ''],
        default_value=[
                get_package_share_directory('barista_robot_description'), 
                '/worlds/playpen.world'
            ],
        description="Path to Gazebo SDF world file"
    )

    # Gazebo launch arguments
    gazebo_launch_args = {
        "verbose" : "false",
        "pause"   : "false",
        "world"   : LaunchConfiguration("world")
    
    }
    
    # Gazebo launch file description with launch args
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
            # launch_arguments=gazebo_launch_args.items(),
        )
    )    

    # Define robot model file to be used
    # urdf_file = 'barista_robot_model.urdf'
    xacro_file = "barista_robot_model.urdf.xacro"
    package_description = "barista_robot_description"
    ####### DATA INPUT END ##########

    print("Fetching robot models ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "xacro", xacro_file)

    robot_name_1 = "rick"
    robot_name_2 = "morty"

    # Robot State Publisher
    # 1. Publishes to topic /robot_description
    # 2. Publishes static tf

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    rsp_robot_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        namespace=robot_name_1,
        emulate_tty=True,
        parameters=[{
            'frame_prefix': robot_name_1 + '/', 
            'use_sim_time': use_sim_time, 
            'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', robot_name_1])
            }],
        output="screen"
    )

    rsp_robot_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        namespace=robot_name_2,
        emulate_tty=True,
        parameters=[{
            'frame_prefix': robot_name_2 + '/', 
            'use_sim_time': use_sim_time, 
            'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', robot_name_2])
            }],
        output="screen"
    )

    # Launch configuration for spawning the robots

    # Position and orientation
    # [X, Y, Z]
    position_1 = [0.0, 0.0, 0.1052]
    position_2 = [-3.0, 3.0, 0.1052]

    # [Roll, Pitch, Yaw]
    orientation_1 = [0.0, 0.0, 0.0]
    orientation_2 = [0.0, 0.0, 0.0]

    # Generate random names
    entity_name_1 = robot_name_1 + "-" + str(int(random.random()*100000))
    entity_name_2 = robot_name_2 + "-" + str(int(random.random()*100000))

    # Spawn robot nodes
    spawn_robot_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity_1',
        output='screen',
        arguments=['-entity',
                   entity_name_1,
                   '-x', str(position_1[0]), 
                   '-y', str(position_1[1]), 
                   '-z', str(position_1[2]),
                   '-R', str(orientation_1[0]), 
                   '-P', str(orientation_1[1]), 
                   '-Y', str(orientation_1[2]),
                   '-topic', 
                   robot_name_1 + '/robot_description'
                   ]
    )

    spawn_robot_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity_2',
        output='screen',
        arguments=['-entity',
                   entity_name_2,
                   '-x', str(position_2[0]), 
                   '-y', str(position_2[1]), 
                   '-z', str(position_2[2]),
                   '-R', str(orientation_2[0]), 
                   '-P', str(orientation_2[1]), 
                   '-Y', str(orientation_2[2]),
                   '-topic', 
                   robot_name_2 + '/robot_description'
                   ]
    )

    # RVIZ Configuration
    rviz_config_dir = \
        os.path.join(get_package_share_directory(package_description), 'rviz', 'urdf_vis_two_robots.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])


    # create and return launch description object
    return LaunchDescription([        
            world_file_arg,
            gazebo,
            rsp_robot_1,
            rsp_robot_2,
            spawn_robot_1,
            spawn_robot_2,
            rviz_node,
    ])