.sh 是 Shell 脚本文件，可以理解为“把多条终端命令保存成一个快捷启动脚本”。

  例如 real_nav.sh 中通常会写：

  ros2 launch rm_nav_bringup bringup_real.launch.py \
      world:=0402 \
      mode:=nav

  它相当于帮你自动在终端执行这条较长的 ROS 2 启动命令。

  常见运行方式：

  cd 26Navigation-master
  bash sh/real_nav.sh

  如果文件有执行权限，也可以：

  ./sh/real_nav.sh

  整个调用关系可以理解为：

  real_nav.sh
      ↓ 执行 ros2 launch
  bringup_real.launch.py
      ↓ 根据 world:=0402 选择地图
  map/0402.yaml、0402.pgm、0402.posegraph
  PCD/0402.pcd

  所以：

  - CMakeLists.txt：规定安装哪些目录
  - .sh：提供一键启动命令和启动参数
  - .launch.py：真正组织、启动各个 ROS 2 节点
  - world:=0402：选择使用 0402 这一套地图
