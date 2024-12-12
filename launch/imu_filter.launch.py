import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory('raptor_robot_v2'), 'config')

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter',
                output='screen',
                remappings=[
                    # ('/d435i/camera/imu', '/imu/data_raw')
                    ('/imu_6050_data_raw', '/imu/data_raw'),
                ],
                parameters=[os.path.join(config_dir, 'imu_filter.yaml')],
            )
        ]
    )
