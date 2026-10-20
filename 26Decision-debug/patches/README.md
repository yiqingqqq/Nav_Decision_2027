# 全向导航朝向策略

决策端已实现 `SendGoal.heading_policy`：

- `hold`（默认）：发送新目标时，从共享的 map-frame `current_pose` 捕获 yaw；此目标期间保持这个参考值，不随行为树 tick 重新捕获。
- `explicit`：使用目标的 yaw，包括有效的零度朝向；隧道准备点采用该策略。
- 普通目标仍使用 position goal checker；准备点使用 general goal checker，隧道路径保留语义地图中的显式朝向。

## 尚未应用的导航补丁

`nav_goal_heading.patch` 以当前工作树为基准，保留导航源码中已有的修改。由于根目录 AGENTS.md 只允许修改 26Decision-debug，这份补丁尚未应用到 26Navigation-master，也未编译或实车验证。当前决策端的改动只解决目标朝向的表达，尚不能完整保证沿途保持朝向。

补丁修改 TEB 的四个文件，增加默认关闭的 `goal_heading_weight`：启用后，在控制器接收全局路径时，用最终目标 yaw 替换中间点默认 yaw；在优化图中增加沿途朝向残差，并在到点原地转向时清除旧局部轨迹显示。它是受速度、加速度约束的软跟踪，不是瞬时锁死姿态；尤其需要验证狭窄地形中的实际跟踪误差。

应用前只读检查：`git apply --check 26Decision-debug/patches/nav_goal_heading.patch`。

获得导航目录修改授权、应用并编译补丁、重启导航后，启动脚本需在决策启动前为 FollowPath、TunnelController 分别设置：

```text
global_plan_overwrite_orientation = false
goal_heading_weight = 100.0
```

100.0 是待仿真验证的起始权重。启动脚本当前尚未加入新参数设置，避免向旧版 TEB 写入未声明或无效的参数。
