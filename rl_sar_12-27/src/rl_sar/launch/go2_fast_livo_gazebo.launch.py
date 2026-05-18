from pathlib import Path

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    wname = LaunchConfiguration("wname")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    launch_dir = Path(__file__).resolve().parent
    workspace_dir = launch_dir.parents[3]
    gazebo_launch_path = launch_dir / "gazebo.launch.py"

    try:
        fast_livo_share = Path(get_package_share_directory("fast_livo"))
        fast_livo_launch_path = fast_livo_share / "launch" / "mapping_go2_gazebo.launch.py"
        if not fast_livo_launch_path.exists():
            raise PackageNotFoundError("fast_livo mapping_go2_gazebo.launch.py")
    except PackageNotFoundError:
        fast_livo_launch_path = (
            workspace_dir
            / "src"
            / "ws_livox"
            / "src"
            / "FAST-LIVO2-ROS2-MID360-Fisheye"
            / "launch"
            / "mapping_go2_gazebo.launch.py"
        )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_launch_path)),
        launch_arguments={
            "rname": "go2",
            "wname": wname,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    fast_livo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(fast_livo_launch_path)),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz": use_rviz,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "wname",
            description="World name in rl_sar/worlds (e.g., stairs, earth, urban)",
            default_value=TextSubstitution(text="stairs"),
        ),
        DeclareLaunchArgument(
            "use_rviz",
            description="Whether to launch Rviz2 for FAST-LIVO2",
            default_value=TextSubstitution(text="False"),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            description="Use /clock from Gazebo for Gazebo, robot_state_publisher and FAST-LIVO2",
            default_value=TextSubstitution(text="true"),
        ),
        gazebo_launch,
        TimerAction(period=5.0, actions=[fast_livo_launch]),
    ])
