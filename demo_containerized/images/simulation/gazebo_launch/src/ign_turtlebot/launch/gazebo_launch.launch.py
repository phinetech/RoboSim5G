import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='turtlebot3_world')

    sdf = os.path.join(
    get_package_share_directory('ign_turtlebot'),
    'models', 'turtlebot3', 'model.sdf')

    doc = xacro.parse(open(sdf))
    xacro.process_doc(doc)

    
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',value=[
        os.path.join("/opt/ros/humble", "share"),
        ":" +
        os.path.join(get_package_share_directory('ign_turtlebot'), "models")])
    package_path = get_package_share_directory("ign_turtlebot")
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = os.path.join(package_path, 'phine_models')
    os.environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] = os.path.join(package_path, 'phine_plugins')
    os.environ['IGN_GUI_PLUGIN_PATH'] = os.path.join(package_path, 'gui_plugins')

    # Spawn robot
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-entity', 'waffle',
                   '-name', 'waffle',
                   '-file', PathJoinSubstitution([
                        get_package_share_directory('ign_turtlebot'),
                        "models", "turtlebot3", "model.sdf"]),
                   '-allow_renaming', 'true',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.01'],
        )
    
    # Spawn world
    ignition_spawn_world = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-file', PathJoinSubstitution([
                        get_package_share_directory('ign_turtlebot'),
                         "worlds", "no_roof_small_warehouse.sdf"]),
                   '-allow_renaming', 'false'],
        )
    
    robot_state_publisher=Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': doc.toxml()}])

    world = os.path.join(get_package_share_directory('ign_turtlebot'), "worlds", "world_only.sdf")

    return LaunchDescription([
        ign_resource_path,
        ignition_spawn_entity,
        ignition_spawn_world,
        robot_state_publisher,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('ign_args', [' -r -v 3 ' +
                              world
                             ])]),
                             
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),

        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'),
            DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
    ])
