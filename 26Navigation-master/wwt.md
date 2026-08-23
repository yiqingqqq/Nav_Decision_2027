
```
    nohup  ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=YOUR_WORLD_NAME \
    mode:=mapping  \
    lio:=fastlio \
    lio_rviz:=False \
    nav_rviz:=false \
    > ./log.log &
```   
```
    nohup ros2 launch rm_serial_driver rm_serial_driver.launch.py &
```
```
ps -ef | grep bringup_real
ps -ef |grep rm_serial_driver
```
```sh
pkill -9 -f ros2
pkill -9 -f nav2
pkill -9 -f fastlio
pkill -9 -f livox
pkill -9 -f slam
pkill -9 -f pointcloud_to_laserscan
pkill -9 -f robot_state_publisher
pkill -9 -f ground_segmentation
pkill -9 -f static_transform_publisher
```


m123@rm:~/Desktop/shaobing$ ros2 topic list
/clicked_point
/downsampled_costmap
/downsampled_costmap_updates
/global_costmap/costmap
/global_costmap/costmap_updates
/global_costmap/voxel_grid
/global_costmap/voxel_marked_cloud
/initialpose
/local_costmap/costmap
/local_costmap/costmap_updates
/local_costmap/published_footprint
/local_costmap/voxel_marked_cloud
/local_plan
/map
/map_metadata
/map_updates
/mobile_base/sensors/bumper_pointcloud
/parameter_events
/particle_cloud
/plan
/pose
/robot_description
/rosout
/scan
/slam_toolbox/feedback
/slam_toolbox/graph_visualization
/slam_toolbox/scan_visualization
/slam_toolbox/update
/teb_markers
/teb_markers_array
/teb_poses
/tf
/tf_static
/waypoints



4.812;-2.130
5.732, -2.644
4.660, -3.326,