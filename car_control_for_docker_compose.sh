#!/bin/bash
source /opt/ros/humble/setup.bash
colcon build
. ./install/setup.bash
# ros2 run pros_car_py carB_keboard
ros2 run pros_car_py carB_writer &
ros2 run pros_car_py arduino_arm_writer &
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
tail -f /dev/null