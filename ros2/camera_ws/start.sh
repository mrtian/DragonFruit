source ~/camera_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
nohup taskset -c 4,5,6 ros2 launch astra_camera astra.launch.xml depth_fps:=5 color_fps:=10 enable_point_cloud:=true >> astra.log &
nohup ros2 run image_transport republish raw compressed --ros-args --remap in:=/camera/color/image_raw --remap out/compressed:=/camera/color/image_raw/compressed >> camera_compressed.log &

