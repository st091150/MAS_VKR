"""Запуск симуляции Gazebo: маршрут по waypoints, RViz, навигация."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from thrust_nav_sim.gazebo_waypoint_world import generate_gazebo_world


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('thrust_nav_sim')
    nav_pkg = get_package_share_directory('navigation_ros')
    bridge = os.path.join(pkg, 'config', 'bridge.yaml')
    nav_params = os.path.join(nav_pkg, 'config', 'nav_params.yaml')
    waypoints = os.path.join(nav_pkg, 'config', 'waypoints.json')
    base_world = os.path.join(pkg, 'worlds', 'thrust_world.sdf')
    rviz_cfg = os.path.join(pkg, 'rviz', 'waypoint_nav.rviz')
    robot_sdf = os.path.join(pkg, 'sdf', 'thrust_robot.sdf')

    st = {'use_sim_time': True}

    def make_gz_sim(context):
        gz_args_raw = LaunchConfiguration('gz_args').perform(context).strip()
        if gz_args_raw:
            gz_args = gz_args_raw
        else:
            waypoint_file = LaunchConfiguration('waypoint_file').perform(context)
            try:
                world = generate_gazebo_world(base_world, waypoint_file)
                print(f'[sim.launch.py] Сгенерирован мир Gazebo: {world}')
            except Exception as exc:
                raise RuntimeError(f'Ошибка генерации мира Gazebo из маршрута: {exc}') from exc
            world_gz = world.replace(os.sep, os.altsep) if os.altsep else world
            gz_args = f'{world_gz} -r -v 1'
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
                ),
                launch_arguments={'gz_args': gz_args}.items(),
            )
        ]

    def unpause() -> ExecuteProcess:
        return ExecuteProcess(
            cmd=[
                'gz', 'service', '-s', '/world/thrust_world/control',
                '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                '--timeout', '8000', '--req', 'pause: false',
            ],
            output='screen',
        )

    wp_nav = Node(
        package='navigation_ros',
        executable='waypoint_nav',
        parameters=[
            st,
            nav_params,
            {'waypoint_file': LaunchConfiguration('waypoint_file')},
        ],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[st],
        output='screen',
    )

    unpause_if = LaunchConfiguration('auto_unpause_service')
    unpauses = [
        TimerAction(period=d, actions=[unpause()], condition=IfCondition(unpause_if))
        for d in (4.0, 9.0)
    ]

    def schedule_spawn_when_ready(context):
        gz_args_raw = LaunchConfiguration('gz_args').perform(context).strip()
        wf = LaunchConfiguration('waypoint_file').perform(context)
        rsdf = LaunchConfiguration('robot_sdf').perform(context)
        wait_loaded = LaunchConfiguration('spawn_when_waypoints_loaded').perform(context).strip().lower() in (
            '1', 'true', 'yes', 'on',
        )
        if gz_args_raw:
            wait_loaded = False

        if wait_loaded:
            spawn_cmd = [
                'ros2', 'run', 'thrust_nav_sim', 'spawn_when_world_ready',
                '--waypoint-file', wf,
                '--robot-sdf', rsdf,
                '--world', 'thrust_world',
                '--robot-name', 'thrust_robot',
                '--grace', LaunchConfiguration('spawn_poll_grace_s').perform(context),
                '--poll', LaunchConfiguration('spawn_poll_period_s').perform(context),
                '--timeout', LaunchConfiguration('spawn_world_ready_timeout_s').perform(context),
            ]
        else:
            spawn_cmd = [
                'ros2', 'run', 'thrust_nav_sim', 'spawn_when_world_ready',
                '--waypoint-file', wf,
                '--robot-sdf', rsdf,
                '--world', 'thrust_world',
                '--robot-name', 'thrust_robot',
                '--legacy-sleep', LaunchConfiguration('spawn_after_s').perform(context),
            ]

        bridge_n = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge, 'use_sim_time': True}],
            output='screen',
        )

        spawn_exec = ExecuteProcess(cmd=spawn_cmd, output='screen')
        after_spawn = RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_exec,
                on_exit=[
                    TimerAction(period=LaunchConfiguration('bridge_delay_s'), actions=[bridge_n]),
                    TimerAction(period=LaunchConfiguration('waypoint_nav_delay_s'), actions=[wp_nav]),
                ],
            )
        )
        return [spawn_exec, after_spawn]

    return LaunchDescription(
        [
            DeclareLaunchArgument('gz_args', default_value=''),
            DeclareLaunchArgument('auto_unpause_service', default_value='true'),
            DeclareLaunchArgument('spawn_when_waypoints_loaded', default_value='true'),
            DeclareLaunchArgument('spawn_poll_grace_s', default_value='2.0'),
            DeclareLaunchArgument('spawn_poll_period_s', default_value='0.75'),
            DeclareLaunchArgument('spawn_world_ready_timeout_s', default_value='300.0'),
            DeclareLaunchArgument('spawn_after_s', default_value='15.0'),
            DeclareLaunchArgument('bridge_delay_s', default_value='1.0'),
            DeclareLaunchArgument(
                'waypoint_nav_delay_s',
                default_value='5.0',
                description='Задержка waypoint_nav после старта моста GPS/IMU.',
            ),
            DeclareLaunchArgument('waypoint_file', default_value=waypoints),
            DeclareLaunchArgument('enable_rviz', default_value='true'),
            DeclareLaunchArgument('rviz_config', default_value=rviz_cfg),
            DeclareLaunchArgument('robot_sdf', default_value=robot_sdf),
            OpaqueFunction(function=make_gz_sim),
            rviz,
            *unpauses,
            OpaqueFunction(function=schedule_spawn_when_ready),
        ]
    )
