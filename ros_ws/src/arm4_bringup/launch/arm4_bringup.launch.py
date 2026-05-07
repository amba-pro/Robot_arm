import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")
    rqt = LaunchConfiguration("rqt")
    project_root = LaunchConfiguration("project_root")
    start_robot_server = LaunchConfiguration("start_robot_server")
    start_angle_reader = LaunchConfiguration("start_angle_reader")

    arm4_pkg_share = get_package_share_directory("arm4_bringup")
    rviz_config = PathJoinSubstitution([arm4_pkg_share, "rviz", "arm4.rviz"])

    cache_file = PathJoinSubstitution([project_root, "angles_cache.json"])
    angle_reader_script = PathJoinSubstitution([project_root, "angle_reader.py"])
    robot_server_script = PathJoinSubstitution([project_root, "robot_tcp_server.py"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("rqt", default_value="false"),
            DeclareLaunchArgument("start_robot_server", default_value="false"),
            DeclareLaunchArgument("start_angle_reader", default_value="false"),
            DeclareLaunchArgument("project_root", default_value=os.getcwd()),
            Node(
                package="arm4_bringup",
                executable="angle_cache_publisher",
                name="angle_cache_publisher",
                output="screen",
                parameters=[
                    {"cache_file": cache_file},
                    {"publish_rate_hz": 10.0},
                ],
            ),
            ExecuteProcess(
                condition=IfCondition(start_angle_reader),
                cmd=["python3", angle_reader_script],
                name="angle_reader_py",
                output="screen",
            ),
            ExecuteProcess(
                condition=IfCondition(start_robot_server),
                cmd=["python3", robot_server_script],
                name="robot_tcp_server_py",
                output="screen",
            ),
            ExecuteProcess(
                condition=IfCondition(rviz),
                cmd=["rviz2", "-d", rviz_config],
                name="rviz2",
                output="screen",
            ),
            ExecuteProcess(
                condition=IfCondition(rqt),
                cmd=["rqt"],
                name="rqt",
                output="screen",
            ),
        ]
    )
