import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Cesta k RViz configu
    rviz_config_dir = os.path.join(
        get_package_share_directory('mpc_rbt_student'),
        'rviz',
        'config.rviz')

    return LaunchDescription([
        # Spuštění uzlu pro výpočet odometrie
        Node(
            package='mpc_rbt_student',
            executable='localization_node',
            name='localization_node',
            output='screen'
        ),
        # Spuštění graf. rozhraní RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )
    ])
