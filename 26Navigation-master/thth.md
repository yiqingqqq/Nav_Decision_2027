## [***<ins>局域网下ROS节点怎么共享***](~/.bashrc)
1. WSL中启动rviz2
    - [.wslconfig]("C:\Users\31320\.wslconfig"): 配置镜像网卡
    - 关闭防火墙
```sh
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=3
```
2. 启动 `foxglove_bridge`
```sh
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```