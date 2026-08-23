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

隧道模式基于 `v2.xml`，不会改变 `v0.xml`、`v1.xml` 或 `v2.xml`：

```bash
chmod +x run_tunnel.sh
./run_tunnel.sh
```

该模式当前仅使用红方坐标，不订阅或依赖裁判系统的 `/feedback_robot_id`。

`run.sh` 启动普通 `v2.xml` 模式，`run_tunnel.sh` 启动 `tunnel.xml` 模式，二者不要同时运行。

`run_tunnel.sh` 会等待 Nav2 的 `/navigate_to_pose` 可用后再启动决策，避免导航尚未就绪时反复报 Action Server 不可达。

### 一键启动 RMUL2027 隧道仿真验收

验收脚本使用 `RMUL2027.posegraph` 和 slam_toolbox localization，并依次启动仿真、Nav2 和 RViz。该序列化地图以出生位姿为局部 `map` 原点，所以场地绝对坐标会转换为导航工程坐标。地图与 Nav2 就绪后，脚本会直接执行一次控制器验收路线：

```text
FollowPath 到局部 map 原点 (0.0, 0.0)
→ FollowPath 到较近端点 (1.0, 0.6)
→ 切换 TunnelController
→ 到达隧道内部强制点 (2.5, 0.6)
→ 驶向另一端 (4.5, 0.6)
→ 恢复 FollowPath
```

上述工程坐标分别对应场地绝对坐标 `(5.0,-3.0) → (4.0,-3.6) → (2.5,-3.6) → (0.5,-3.6)`。

```bash
chmod +x run_tunnel_sim.sh
./run_tunnel_sim.sh
```

隧道内部点用于约束全局路径必须经过隧道，防止规划器因为隧道膨胀代价较高而改走大路。

完整策略不会永久指定入口和出口。开始一次穿越时，`SelectTunnelDirection` 根据机器人当前位置选择较近端点作为入口，另一端作为出口，并将该方向锁定到本次穿越结束。

完整隧道策略使用独立的 `NonBlockingRateController` 周期发布旋转状态；等待下一个发布周期时返回成功，不会再由外层 `ReactiveSequence` 中断正在运行的 `SendGoal`。原 `v0.xml`、`v1.xml`、`v2.xml` 继续使用原有 `RateController`。

`RunOnceUntilHalted` 保证一次任务只穿越一次；撤退、补血恢复或比赛分支切换使本轮任务结束后会重置，下一轮重新按当前位置选择近端。

该脚本不运行完整比赛策略树，避免其他决策分支干扰控制器验收。验收结束后 RViz 会保持打开；按 `Ctrl+C` 会关闭该脚本启动的全部子进程。
脚本会主动解除 Gazebo 物理暂停，避免因仿真时钟不运行而无法加载 `/map`。

### 杀死进程
lsof -i :1667
kill -9 

## 功能实现

1.高转速=4.5±0.5，低转速=0，转速变换时平滑

2.血量低于110发Supply(-0.25,0.0)，撤退后必须回血到400重新进入进攻分支

3.进入进攻分支后，发Attack(6.0,-3.75)

4.发Attack返回失败，发Center(5.0,-3.0)，随机移动move=True，边长length=1

5.在Center附近2.0m以内，发高转速，否则发低转速

6.扣血判定：若扣血，接下来3秒内返回成功。若扣血判定成功，发Back(5.0,-4.0)，随机移动move=True，边长length=1
