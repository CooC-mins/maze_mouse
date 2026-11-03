import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_maze = get_package_share_directory('maze_mouse')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    world_file = os.path.join(pkg_maze, 'worlds', 'maze_world.world')
    rviz_config = os.path.join(pkg_maze, 'rviz', 'maze.rviz')

    # 1. 启动 Gazebo（服务端+客户端），加载自定义迷宫世界
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file}.items(),
    )

    # 2. TurtleBot3 机器人状态发布（读取环境变量 TURTLEBOT3_MODEL=burger）
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # 3. 在迷宫入口外生成 Burger，-Y 1.5708 使车头朝北（正对入口）
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'burger',
                   '-file', os.path.join(pkg_tb3_gazebo, 'models',
                                         'turtlebot3_burger', 'model.sdf'),
                   '-x', '1.4', '-y', '-0.8', '-z', '0.01', '-Y', '1.5708'],
        output='screen',
    )

    # 4. RViz2 可视化（加载预设配置：模型、TF、激光点云）
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(gazebo)               # Gazebo 仿真环境
    ld.add_action(robot_state_publisher)  # 发布机器人 URDF 与 TF
    ld.add_action(spawn_robot)          # 生成小车
    ld.add_action(rviz)                 # 可视化
    return ld
