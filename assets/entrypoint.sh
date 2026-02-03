#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
if [ -f "$HOME/catkin_ros/devel/setup.bash" ]; then
  source "$HOME/catkin_ros/devel/setup.bash"
fi

exec "$@"
