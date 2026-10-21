# 全向导航朝向策略

决策端已实现 `SendGoal.heading_policy`：

- `hold`（默认）：发送新目标时，从共享的 map-frame `current_pose` 捕获 yaw；此目标期间保持这个参考值，不随行为树 tick 重新捕获。
- `explicit`：使用目标的 yaw，包括有效的零度朝向，用于明确要求朝向的特殊任务。
- 普通目标和准备点均使用 hold 与 position goal checker；进入准备区域后，入口与隧道路径使用语义地图中的显式朝向。

## 导航端实现

2026-10-21 获得用户对 4 个 TEB 文件的修改授权后，导航端实现已应用到 `26Navigation-master`。`nav_goal_heading.patch` 仅保留为最初提案的历史记录，不要再次应用；以实际源码为准。

实际实现的 `goal_heading_weight` 默认值为 100.0，让普通导航默认使用决策端捕获的朝向。控制器接收全局路径时，用最终目标 yaw 替换中间点默认 yaw；初始化时优先保持该朝向；优化图增加沿途朝向残差；到点原地转向时清除旧局部轨迹显示。设为 0.0 可恢复旧行为。它是受速度、加速度约束的软跟踪，需要验证狭窄地形中的实际跟踪误差。

重新编译后须重启导航进程才能加载新库。`run_tunnel.sh` 在决策启动前检查新参数是否存在，并为 FollowPath、TunnelController 分别设置：

```text
global_plan_overwrite_orientation = false
goal_heading_weight = 100.0
```

如果导航仍加载旧库，脚本会报错并停止启动决策，不再静默忽略参数设置失败。

优化器回归测试在 `rm_behavior_tree/test/teb_heading`，独立构建，使用隔离 ROS 域运行，不发送机器人动作目标：

```bash
source /opt/ros/humble/setup.bash
source 26Navigation-master/install/setup.bash
cmake -S 26Decision-debug/rm_behavior_tree/test/teb_heading -B /tmp/teb-heading-regression
cmake --build /tmp/teb-heading-regression -j2
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 /tmp/teb-heading-regression/test_teb_heading
```
