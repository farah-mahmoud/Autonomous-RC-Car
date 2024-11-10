import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import launch_ros

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('race_it'))
    
    #xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    xacro_file = os.path.join(pkg_path,'description', 'race_car.xacro')
    
    robot_description_config = xacro.process_file(xacro_file)
    
    default_rviz_config_path = os.path.join(pkg_path, 'config/rviz/urdf_config.rviz')
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    
        
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time' : use_sim_time}]
    )
    

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
    )

    
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])#,
    #        launch_arguments={'world': "my_bot/worlds/obstacles.world"}.items()
    )
    
    
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', '', '-topic', 'robot_description'],
        output='screen'
    )
    
    ackermann_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["asc"],
)

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),

        node_robot_state_publisher,
        joint_state_publisher_node,
        rviz_node,
        gazebo,
        spawn_entity,
        ackermann_spawner
    ])
