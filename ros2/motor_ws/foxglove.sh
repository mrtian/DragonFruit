source ~/camera_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
nohup taskset -c 0,1 ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
