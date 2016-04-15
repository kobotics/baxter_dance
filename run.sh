roslaunch freenect_launch freenect.launch rgb_frame_id:=camera_rgb_optical_frame depth_frame_id:=camera_depth_optical_frame & 
sleep 7 && roslaunch my_pcl_tutorial_v2 perception.launch & 
sleep 9 && rosrun tf static_transform_publisher 0.15 0 0.9 0.15 0.5 0 /torso /camera_link 50 &
sleep 9.5 && rosrun my_pcl_tutorial_v2 xyzavg_ros input:=/voxel_grid/output &
sleep 10 && rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 /base_link /camera_link 1000 &

