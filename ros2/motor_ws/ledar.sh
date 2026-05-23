export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml

# 命令标准格式：x y z yaw pitch roll 父坐标系 子坐标系
nohup ros2 run tf2_ros static_transform_publisher 0.32 0.0 0.05 0.0 0.0 0.0 camera_link laser_frame >> tf2.log &&
nohup python3 yds2md_radar_node.py >>ledar.log & 
