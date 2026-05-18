# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

from launch.actions import TimerAction

def generate_launch_description():
    rname = LaunchConfiguration("rname")
    wname = LaunchConfiguration("wname")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rl_sar_share = get_package_share_directory("rl_sar")

    robot_name = ParameterValue(Command(["echo -n ", rname]), value_type=str)
    ros_namespace = ParameterValue(Command(["echo -n ", "/", rname, "_gazebo"]), value_type=str)
    gazebo_model_name = ParameterValue(Command(["echo -n ", rname, "_gazebo"]), value_type=str)
    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            PathJoinSubstitution([rl_sar_share, "models"]),
            ":",
            EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
        ],
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            Command(["echo -n ", Command(["ros2 pkg prefix ", rname, "_description"])]),
            "/share/", rname, "_description/xacro/robot.xacro"
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": use_sim_time},
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            # "verbose": "true",
            # "pause": "true",  # Not Available
            "world": PathJoinSubstitution([rl_sar_share, "worlds", [wname, ".world"]]),
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "robot_model",
            "-z", "1.0",
        ],
        output="screen",
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # --- 延迟启动 joint_state_broadcaster ---
    # joint_state_broadcaster_node = TimerAction(
    #     period=5.0,  # 延迟 5 秒
    #     actions=[
    #         Node(
    #             package="controller_manager",
    #             executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
    #             arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    #             output="screen",
    #         )
    #     ],
    # )

    robot_joint_controller_node = Node(
        package="controller_manager",
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        arguments=["robot_joint_controller"],
        output="screen",
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.1,
            'autorepeat_rate': 0.0,
        }],
    )

    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        parameters=[{
            "robot_name": robot_name,
            "gazebo_model_name": gazebo_model_name,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rname",
            description="Robot name (e.g., a1, go2)",
            default_value=TextSubstitution(text=""),
        ),
        DeclareLaunchArgument(
            "wname",
            description="World name in rl_sar/worlds (e.g., stairs, earth, urban)",
            default_value=TextSubstitution(text="stairs"),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            description="Use /clock from Gazebo",
            default_value=TextSubstitution(text="true"),
        ),
        gazebo_model_path,
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_node,
        # robot_joint_controller_node,  # Spawn in rl_sim.cpp
        joy_node,
        param_node,
    ])
