from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Gazebo起動をlaunchファイルから呼び出し
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={'verbose': 'true'}.items()
    )

    robot_description = Command([
        'cat ',
        PathJoinSubstitution([FindPackageShare('ros2_gazebo_object_detection'), 'urdf', 'lider.urdf'])
    ])

    # robot_state_publisherノード
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # URDFファイルのパス(camera)
    lider_urdf_file = os.path.join(
        get_package_share_directory('ros2_gazebo_object_detection'),  # ←自分のパッケージ名
        'urdf',
        'lider.urdf'
    )
    # URDFファイルのパス(subject)
    subject_urdf_file = os.path.join(
        get_package_share_directory('ros2_gazebo_object_detection'),  # ←自分のパッケージ名
        'urdf',
        'subject.urdf'
    )

    # URDFファイルのパス(subject)
    subject2_urdf_file = os.path.join(
        get_package_share_directory('ros2_gazebo_object_detection'),  # ←自分のパッケージ名
        'urdf',
        'subject2.urdf'
    )

    # rviz_configファイルのパス
    rviz_config_file = os.path.join(
        get_package_share_directory('ros2_gazebo_object_detection'),
        'rviz',
        'lider_config.rviz'
    )

    spawn_camera_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'lider',   # Gazebo 上での名前
            '-file', lider_urdf_file      # URDF ファイルパス
        ],
        output='screen'
    )

    spawn_subject_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'subject',   # Gazebo 上での名前
            '-file', subject_urdf_file      # URDF ファイルパス
        ],
        output='screen'
    )


    spawn_subject2_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'subject2',   # Gazebo 上での名前
            '-file', subject2_urdf_file      # URDF ファイルパス
        ],
        output='screen'
    )

    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],  # 設定ファイルがあれば指定
            output='screen'
        )

    # 自作ノード起動例
    camera_node = Node(
        package='ros2_gazebo_object_detection',
        executable='get_image_node',
        name='get_image_node',
        output='screen'
    )

#ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_scan
#korewo zikkousuruto umakuiku

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_camera_entity_node,
        spawn_subject_entity_node,
        spawn_subject2_entity_node,
        rviz2,
        camera_node])