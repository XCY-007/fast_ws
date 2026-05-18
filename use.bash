#!/usr/bin/env bash
set -e

ROOT_DIR="/home/xcy/SLAM_Project/fast_ws0"
RL_WS="${ROOT_DIR}/rl_sar_12-27"
LIVO_WS="${ROOT_DIR}/src/ws_livox"

clean_conda_env_for_ros() {
  export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
  unset PYTHONPATH PYTHONHOME
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
}

source_ros() {
  clean_conda_env_for_ros
  source /opt/ros/humble/setup.bash
}

source_rl() {
  source_ros
  source "${RL_WS}/install/setup.bash"
}

source_livo() {
  source_ros
  source "${LIVO_WS}/install/setup.bash"
}

usage() {
  cat <<EOF
Usage:
  bash ${ROOT_DIR}/use.bash gazebo   # open Gazebo + Go2 model
  bash ${ROOT_DIR}/use.bash dog      # start Go2 RL controller
  bash ${ROOT_DIR}/use.bash slam     # start FAST-LIVO2 + RViz
  bash ${ROOT_DIR}/use.bash check    # print ROS package paths

Start order:
  1) gazebo
  2) dog
  3) slam
EOF
}

cmd="${1:-help}"

case "${cmd}" in
  gazebo|sim)
    source_rl
    source /usr/share/gazebo/setup.bash
    exec ros2 launch rl_sar gazebo.launch.py rname:=go2
    ;;

  dog|robot|control)
    source_rl
    exec ros2 run rl_sar rl_sim
    ;;

  slam|mapping)
    source_livo
    exec ros2 launch "${LIVO_WS}/src/FAST-LIVO2-ROS2-MID360-Fisheye/launch/mapping_go2_gazebo.launch.py" use_rviz:=True
    ;;

  check)
    source_rl
    source "${LIVO_WS}/install/setup.bash"
    echo "ROS_PACKAGE_PATH:"
    ros2 pkg prefix rl_sar
    ros2 pkg prefix robot_joint_controller
    ros2 pkg prefix fast_livo
    ;;

  help|-h|--help)
    usage
    ;;

  *)
    echo "Unknown command: ${cmd}" >&2
    usage >&2
    exit 2
    ;;
esac
