import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')

    localization_node = Node(
        package='mpc_rbt_student',
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    planning_node = Node(
        package='mpc_rbt_student',
        executable='planning_node',
        name='planning_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    motion_control_node = Node(
        package='mpc_rbt_student',
        executable='motion_control_node',
        name='motion_control_node',
        output='screen'
    )   

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        localization_node,
        planning_node,
        motion_control_node,
        rviz_node
    ])