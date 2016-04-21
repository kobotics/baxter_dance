roslaunch my_pcl_tutorial_v2 SAC.launch &
rosrun tf static_transform_publisher 0.15 0 0.9 0.15 0.5 0 /torso /camera_link 50 &
sleep 1 && rosrun my_pcl_tutorial_v2 xyzavg_ros input:=/move_group/self_filtered_cloud &
sleep 2 && rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 /base_link /camera_link 1000 &

