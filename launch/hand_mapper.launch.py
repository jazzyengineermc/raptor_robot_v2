import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('raptor_robot_v2'))
    xacro_file = os.path.join(pkg_path,'description','hand_mapper.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Create RealSense d435i Camera node
    node_d435i_camera = Node(
        package='raptor_robot_v2',
        executable='rs_launch.py',
        output='screen'
    )

    # IMU filter node
    tf_imu_camera = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "camera_imu_optical_frame", "camera_link"]
    )

    # RGBD Odometry and Map node
    node_rtabmap = Node(
        package='raptor_robot_v2',
        executable='rtabmap.launch.py',
        output='screen'
    )

    # RViz Node
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', 'config/hhmapper.rviz']
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        node_robot_state_publisher,
        node_d435i_camera,
        tf_imu_camera,
        node_rtabmap,
        # node_rviz
    ])
