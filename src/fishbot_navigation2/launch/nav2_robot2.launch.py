import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = 'robot2'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')

    map_yaml_path = LaunchConfiguration(
        'map',
        default=os.path.join(
            fishbot_navigation2_dir,
            'maps',
            'queue_warehouse_map.yaml'
        )
    )

    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            fishbot_navigation2_dir,
            'config',
            'nav2_params_robot2.yaml'
        )
    )

    nav2_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': 'True',
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'slam': 'False'
        }.items()
    )

    return launch.LaunchDescription([
        nav2_launch
    ])