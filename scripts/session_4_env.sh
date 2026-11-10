#!/usr/bin/env bash
# bash / zsh 공통: 프로젝트 루트에서 source scripts/session_4_env.sh
_orda_session4_env() {
  local project_dir script_path ros_setup
  if [ -n "${BASH_VERSION:-}" ]; then
    script_path="${BASH_SOURCE[0]}"
  elif [ -n "${ZSH_VERSION:-}" ]; then
    script_path="${(%):-%x}"
  else
    echo 'bash 또는 zsh에서 실행하세요.' >&2
    return 1
  fi
  project_dir="$(cd -- "$(dirname -- "$script_path")/.." && pwd)"
  if [ ! -x "$project_dir/.venv/bin/python" ]; then
    echo 'README의 프로젝트 .venv를 먼저 준비하세요.' >&2
    return 1
  fi
  ros_setup="${SESSION4_ROS_SETUP:-${SESSION3_ROS_SETUP:-}}"
  if [ -z "$ros_setup" ]; then
    if [ "$(uname -s)" = Darwin ]; then
      ros_setup="$HOME/ros2_jazzy/install/setup.bash"
      [ -z "${ZSH_VERSION:-}" ] || ros_setup="$HOME/ros2_jazzy/install/setup.zsh"
    else
      ros_setup=/opt/ros/jazzy/setup.bash
      [ -z "${ZSH_VERSION:-}" ] || ros_setup=/opt/ros/jazzy/setup.zsh
    fi
  fi
  if [ ! -f "$ros_setup" ]; then
    echo "ROS 2 환경이 없습니다: $ros_setup (README / SESSION_3.md 설치 안내 확인)" >&2
    return 1
  fi
  source "$ros_setup" || return
  if [ -n "${ZSH_VERSION:-}" ]; then
    [ ! -f "$project_dir/workspace/session_3/bridge/install/setup.zsh" ] || source "$project_dir/workspace/session_3/bridge/install/setup.zsh" || return
    [ ! -f "$project_dir/install/local_setup.zsh" ] || source "$project_dir/install/local_setup.zsh" || return
  else
    [ ! -f "$project_dir/workspace/session_3/bridge/install/setup.bash" ] || source "$project_dir/workspace/session_3/bridge/install/setup.bash" || return
    [ ! -f "$project_dir/install/local_setup.bash" ] || source "$project_dir/install/local_setup.bash" || return
  fi
  source "$project_dir/.venv/bin/activate" || return
  export ROS_DOMAIN_ID=40 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export SESSION4_ENV_READY=1
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
  unset FASTDDS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE
  export ROS_LOG_DIR="$project_dir/workspace/session_4/log"
  mkdir -p "$ROS_LOG_DIR"
  cd "$project_dir" || return
}
_orda_session4_env "$@"
