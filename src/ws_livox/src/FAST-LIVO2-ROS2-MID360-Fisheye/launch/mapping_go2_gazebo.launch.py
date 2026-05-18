#!/usr/bin/python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = Path(__file__).resolve().parents[1]
    config_file_dir = package_dir / "config"
    rviz_config_file = package_dir / "rviz_cfg" / "fast_livo2.rviz"

    livo_config_cmd = str(config_file_dir / "gazebo_go2_mid360.yaml")
    camera_config_cmd = str(config_file_dir / "camera_go2_gazebo.yaml")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Whether to launch Rviz2",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use /clock from Gazebo",
    )
    livo_config_arg = DeclareLaunchArgument(
        "livo_params_file",
        default_value=livo_config_cmd,
        description="FAST-LIVO2 parameters for Go2 Gazebo simulation",
    )
    camera_config_arg = DeclareLaunchArgument(
        "camera_params_file",
        default_value=camera_config_cmd,
        description="Camera intrinsics for Go2 Gazebo simulation",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    livo_params_file = LaunchConfiguration("livo_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")

    return LaunchDescription([
        use_rviz_arg,
        use_sim_time_arg,
        livo_config_arg,
        camera_config_arg,
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[
                livo_params_file,
                camera_params_file,
                {"use_sim_time": use_sim_time},
            ],
            output="screen",
        ),
        Node(
            package="fast_livo",
            executable="rgbd_pointcloud_node.py",
            name="rgbd_pointcloud",
            parameters=[{
                "use_sim_time": use_sim_time,
                "fx": 461.0,
                "fy": 461.0,
                "cx": 320.0,
                "cy": 240.0,
                "stride": 3,
                "min_depth": 0.25,
                "max_depth": 8.0,
                "output_frame": "camera_init",
            }],
            output="screen",
        ),
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", str(rviz_config_file)],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ])
