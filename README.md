## online:

launch rviz and camera:

'roslaunch realsense2_camera opensource_tracking.launch'

launch detection:

`roslaunch yolov5_ros yolo_v5.launch `

record rosbag:

`rosbag record -O my_bagfile_1.bag /camera/aligned_depth_to_color/camera_info  camera/aligned_depth_to_color/image_raw /camera/color/camera_info /camera/color/image_raw /camera/imu /camera/imu_info /tf_static `

## offline:

launch rviz camera:

`roslaunch realsense2_camera opensource_tracking.launch offline:=true `

launch detection:

`roslaunch yolov5_ros yolo_v5.launch `

play rosbag:

` rosbag play my_bagfile_1.bag --clock`
