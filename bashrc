source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# Setup ROS 2 workspace if it exists
if [ -f /home/ros/ros2_ws/install/setup.bash ]; then
    source /home/ros/ros2_ws/install/setup.bash
fi