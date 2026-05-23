nohup ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video0" >> v4l2_camera.log &
nohup ros2 run image_transport republish raw in:=/image_raw compressed out:=/image_raw/compressed >> video_compressed.log &
