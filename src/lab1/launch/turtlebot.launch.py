from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'lab1'

    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz2_config',
        'rviz2.config.rviz'
    )
    
    slam_params_path = os.path.join(
        get_package_share_directory(package_name),
        'slam_config',
        'slam_params.yaml'
    )

    sim_time_param = {'use_sim_time': True}

    turtlebot = Node(
        package=package_name,
        executable='turtlebot.py',
        parameters=[sim_time_param]
    )

    turtlebot_pose = Node(
        package=package_name,
        executable='turtlebot_pose.py',
        parameters=[sim_time_param]
    )

    icp = Node(
        package=package_name,
        executable='icp_ekf.py',
        parameters=[sim_time_param]
    )

    slam_path = Node(
        package=package_name,
        executable='slam_path.py',
        parameters=[sim_time_param]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[sim_time_param]
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_path,
            sim_time_param
        ]
    )

    static_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments = ['0', '0', '0', '0', '0', '0', 'base_link_icp', 'base_scan'],
    parameters=[sim_time_param]
    )
    
    return LaunchDescription([
        turtlebot,
        turtlebot_pose,
        icp,
        rviz2,
        static_tf_node,
        slam_path,
        slam_toolbox
    ])