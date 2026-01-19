import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    package_name = 'warehouse_real_nav'

    # 1. IMU Driver Node
    # Note: 'imu_frame' parameter must match the link name in URDF
    mpu6050_driver = Node(
        package='mpu6050driver',
        executable='mpu6050driver',
        name='mpu6050driver_node',
        output='screen',
        parameters=[{'imu_frame': 'imu_link'}]
    )

    # 2. EKF Node (Robot Localization)
    # Load the config file we just created
    ekf_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'ekf.yaml'
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    return LaunchDescription([
        mpu6050_driver,
        robot_localization_node
    ])