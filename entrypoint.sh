#!/bin/bash

set -e

source /opt/ros/humble/setup.bash

# If you built your workspace, you might want to source it automatically too:
# if [ -f /home/ros/ros2_ws/install/setup.bash ]; then
#   source /home/ros/ros2_ws/install/setup.bash
# fi

echo "Provided arguments: $@"

exec $@