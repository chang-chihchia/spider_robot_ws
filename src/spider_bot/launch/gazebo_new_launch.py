import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. 設定路徑
    pkg_path = get_package_share_directory('spider_bot')
    urdf_file = os.path.join(pkg_path, 'urdf', 'spider_bot.urdf') 

    # 2. 處理 URDF
    doc = xacro.process_file(urdf_file)
    robot_description = {'robot_description': doc.toxml()}

    # 3. 啟動 Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    world_file_path = os.path.join(pkg_path, 'worlds', 'empty_high_friction.world')

    # 4. 啟動 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path}.items()
    )

    # 5. 在 Gazebo 裡產生機器人（關鍵修改：高度改為 0.1m，讓它一出生就踩地）
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'spider_bot', '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # 6. 控制器加載（改用官方推薦的 Node spawner 形式，運行更穩定）
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_spider_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['spider_leg_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        
        # 當機器人生成成功後，才載入 Broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        
        # 當 Broadcaster 啟動完畢後，才啟動主腿部控制器
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_spider_controller],
            )
        ),
    ])