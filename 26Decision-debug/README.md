# RM Behavior Tree

基于 BehaviorTree.CPP 实现的决策树，用于 RM（RoboMaster）哨兵的自主决策。

## 环境要求

- **操作系统**: Ubuntu 22.04 (Jammy Jellyfish)
- **ROS2 版本**: Humble Hawksbill
- **GCC 版本**: 11 或更高（支持 C++17）

## 编译

colcon build --symlink-install

source /opt/ros/humble/setup.bash
source ./install/setup.bash
## 启动

### 模拟裁判系统

chmod +x publish.sh
./publish.sh

### 普通决策模式

chmod +x run.sh
./run.sh

### RMUL 2027 隧道模式

隧道模式使用独立的 `tunnel.xml`，不会改变 `v0.xml`、`v1.xml` 或 `v2.xml`：

```bash
chmod +x run_tunnel.sh
./run_tunnel.sh
```

该模式当前仅使用红方坐标，不订阅或依赖裁判系统的 `/feedback_robot_id`。

`run.sh` 和 launch 文件默认启动普通策略 `v2.xml`。隧道策略只能通过 `run_tunnel.sh` 启动；该脚本会等待 Nav2 action server 就绪，二者不要同时运行。

`run_tunnel.sh` 会清理旧的 ROS graph daemon 缓存，并同时等待
`bt_navigator` 和 `controller_server` 进入 active，再确认 `/navigate_to_pose`
与 `/follow_path` 各有一个真实 action server，避免只看到残留的 action 名称就过早启动决策。

#### 调试阶段的自动重启策略

当前 launch 明确使用 `respawn=False`。这是隧道调试阶段的有意设置：
进程崩溃后保留第一个异常，避免 launch 反复拉起进程、重复占用
Groot2/ZMQ `1667` 端口，从而掩盖最初的故障。

只有在启动、端口释放和异常处理经过稳定验证后，才建议生产运行恢复
`respawn=True`。即使 Groot2 的 `1667` 端口被占用，决策进程现在也会记录错误并在无 Groot 监控模式下继续执行，不再因 ZMQ 异常退出。

### RMUL2027 隧道坐标与验收路线

RMUL2027 仿真使用 `RMUL2027.posegraph` 和 slam_toolbox localization。该序列化地图以世界位姿 `(5.0,-3.0,π)` 为局部 `map` 原点；决策层按 `p_map = R(-π)(p_world - (5,-3))` 转换场地绝对坐标。手动启动导航、模拟裁判系统和决策后，验收路线为：

```text
FollowPath 到局部 map 原点 (0.0, 0.0)
→ FollowPath 到入口准备点 (1.0, 0.4)，并将 map yaw 对准 0（世界 yaw π）
→ 向 `/follow_path` 一次提交 TunnelController 有序中心线
→ 经隧道内部点 (2.5, 0.4)
→ 完全驶出到 (4.5, 0.4)
→ 恢复 FollowPath
```

上述工程坐标分别对应场地绝对坐标 `(5.0,-3.0) → (4.0,-3.4) → (2.5,-3.4) → (0.5,-3.4)`。

是否走隧道及方向在入口目标的全局路径生成之前确定。入口成功对准后即进入已承诺状态，决策层直接向 controller server 发送完整的有序中心线，不再调用全局规划器，因此不会在隧道内改选大路。该动作也绕过 `bt_navigator` 的恢复树，禁止隧道内旋转恢复；失败时使用相反点序、保持原隧道姿态退回入口准备点。

完整策略不会永久指定入口和出口。开始一次穿越时，`SelectTunnelDirection` 根据机器人当前位置选择较近端点作为入口，另一端作为出口，并将该方向锁定到本次穿越结束。

完整隧道策略使用独立的 `NonBlockingRateController` 周期发布旋转状态；等待下一个发布周期时返回成功，不会再由外层 `ReactiveSequence` 中断正在运行的 `SendGoal`。原 `v0.xml`、`v1.xml`、`v2.xml` 继续使用原有 `RateController`。

`RunOnceUntilHalted` 保证一次任务只穿越一次；撤退、补血恢复或比赛分支切换使本轮任务结束后会重置，下一轮重新按当前位置选择近端。

### Groot2 端口检查

```bash
lsof -nP -iTCP:1667 -sTCP:LISTEN
```

如端口被旧的 `rm_behavior_tree` 或 Groot 进程占用，先正常终止该进程，
确认端口释放后再运行 `run_tunnel.sh`。

## 功能实现

1.高转速=4.5±0.5，低转速=0，转速变换时平滑

2.血量低于110发Supply(-0.25,0.0)，撤退后必须回血到400重新进入进攻分支

3.进入进攻分支后，发Attack(6.0,-3.75)

4.发Attack返回失败，发Center(5.0,-3.0)，随机移动move=True，边长length=1

5.在Center附近2.0m以内，发高转速，否则发低转速

6.扣血判定：若扣血，接下来3秒内返回成功。若扣血判定成功，发Back(5.0,-4.0)，随机移动move=True，边长length=1
