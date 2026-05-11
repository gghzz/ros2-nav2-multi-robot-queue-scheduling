import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    fishbot_description_path = get_package_share_directory('fishbot_description')

    default_model_path = fishbot_description_path + '/urdf/fishbot/fishbot.urdf.xacro'

    robot1_controller_yaml = fishbot_description_path + '/config/robot1_ros2_controller.yaml'
    robot2_controller_yaml = fishbot_description_path + '/config/robot2_ros2_controller.yaml'

    # 当前先用 custom_room.world
    default_world_path = fishbot_description_path + '/world/custom_room.world'

    # 后面做排队作业 world 时再切换成：
    # default_world_path = fishbot_description_path + '/world/queue_warehouse.world'

    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='URDF/xacro file path'
    )

    robot1_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command([
            'xacro ',
            launch.substitutions.LaunchConfiguration('model'),
            ' robot_namespace:=robot1'
        ]),
        value_type=str
    )

    robot2_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command([
            'xacro ',
            launch.substitutions.LaunchConfiguration('model'),
            ' robot_namespace:=robot2'
        ]),
        value_type=str
    )

    # robot1 状态发布器：关键是 frame_prefix
    robot1_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        name='robot1_robot_state_publisher',
        parameters=[
            {
                'robot_description': robot1_description,
                'use_sim_time': True,
                'frame_prefix': 'robot1/'
            }
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        output='screen'
    )

    # robot2 状态发布器：关键是 frame_prefix
    robot2_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot2',
        name='robot2_robot_state_publisher',
        parameters=[
            {
                'robot_description': robot2_description,
                'use_sim_time': True,
                'frame_prefix': 'robot2/'
            }
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        output='screen'
    )

    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch',
            '/gazebo.launch.py'
        ]),
        launch_arguments=[
            ('world', default_world_path),
            ('verbose', 'true')
        ]
    )

    generate_robot1_urdf = launch.actions.ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            f'xacro {default_model_path} robot_namespace:=robot1 > /tmp/robot1.urdf'
        ],
        output='screen'
    )

    generate_robot2_urdf = launch.actions.ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            f'xacro {default_model_path} robot_namespace:=robot2 > /tmp/robot2.urdf'
        ],
        output='screen'
    )

    spawn_robot1_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', '/tmp/robot1.urdf',
            '-entity', 'robot1',
            '-x', '-3.5',
            '-y', '0.5',
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )

    spawn_robot2_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', '/tmp/robot2.urdf',
            '-entity', 'robot2',
            '-x', '-3.5',
            '-y', '-0.5',
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )

    load_robot1_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'controller_manager',
            'spawner',
            'robot1_joint_state_broadcaster',
            '--controller-manager',
            '/robot1/controller_manager',
            '--param-file',
            robot1_controller_yaml
        ],
        output='screen'
    )

    load_robot1_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'controller_manager',
            'spawner',
            'robot1_diff_drive_controller',
            '--controller-manager',
            '/robot1/controller_manager',
            '--param-file',
            robot1_controller_yaml
        ],
        output='screen'
    )

    load_robot2_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'controller_manager',
            'spawner',
            'robot2_joint_state_broadcaster',
            '--controller-manager',
            '/robot2/controller_manager',
            '--param-file',
            robot2_controller_yaml
        ],
        output='screen'
    )

    load_robot2_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'controller_manager',
            'spawner',
            'robot2_diff_drive_controller',
            '--controller-manager',
            '/robot2/controller_manager',
            '--param-file',
            robot2_controller_yaml
        ],
        output='screen'
    )

    return launch.LaunchDescription([
        action_declare_arg_model_path,

        robot1_state_publisher_node,
        robot2_state_publisher_node,

        launch_gazebo,

        # 1. 先生成 robot1.urdf
        generate_robot1_urdf,

        # 2. robot1.urdf 生成完成后，生成 robot2.urdf
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=generate_robot1_urdf,
                on_exit=[
                    generate_robot2_urdf
                ],
            )
        ),

        # 3. robot2.urdf 生成完成后，先 spawn robot1
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=generate_robot2_urdf,
                on_exit=[
                    spawn_robot1_node
                ],
            )
        ),

        # 4. robot1 spawn 完成后，先加载 robot1 joint_state
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_robot1_node,
                on_exit=[
                    launch.actions.TimerAction(
                        period=3.0,
                        actions=[
                            load_robot1_joint_state_controller
                        ]
                    )
                ],
            )
        ),

        # 5. robot1 joint_state 加载完成后，加载 robot1 diff_drive
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_robot1_joint_state_controller,
                on_exit=[
                    launch.actions.TimerAction(
                        period=1.0,
                        actions=[
                            load_robot1_diff_drive_controller
                        ]
                    )
                ],
            )
        ),

        # 6. robot1 diff_drive 加载完成后，再 spawn robot2
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_robot1_diff_drive_controller,
                on_exit=[
                    launch.actions.TimerAction(
                        period=1.0,
                        actions=[
                            spawn_robot2_node
                        ]
                    )
                ],
            )
        ),

        # 7. robot2 spawn 完成后，加载 robot2 joint_state
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_robot2_node,
                on_exit=[
                    launch.actions.TimerAction(
                        period=3.0,
                        actions=[
                            load_robot2_joint_state_controller
                        ]
                    )
                ],
            )
        ),

        # 8. robot2 joint_state 加载完成后，加载 robot2 diff_drive
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_robot2_joint_state_controller,
                on_exit=[
                    launch.actions.TimerAction(
                        period=1.0,
                        actions=[
                            load_robot2_diff_drive_controller
                        ]
                    )
                ],
            )
        ),
    ])