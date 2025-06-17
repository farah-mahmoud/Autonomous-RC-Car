# race_it/launch/hardware_launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg = get_package_share_directory('race_it')
    xacro_file = os.path.join(pkg, 'description', 'race_car.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # Robot state publisher (URDF to TF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # Static transform from base_link → laser_frame
    static_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.1', '0.0', '0.15', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )

    # Joint state publisher (optional testing only)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # Odometry + TF publisher from serial (IMU + encoder)
    serial_odom_publisher_node = Node(
        package='tkio_ros',
        executable='serial_odom_publisher',
        name='serial_odom_publisher',
        output='screen',
    )

    # EKF node (disabled for now)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=['/path/to/ekf.yaml'],
    )

    # Return all active nodes
    return LaunchDescription([
        robot_state_publisher_node,
        static_laser_tf_node,
        serial_odom_publisher_node,
        #joint_state_publisher_node,
        #ekf_node,
    ])
