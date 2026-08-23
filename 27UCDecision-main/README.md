# WHUT-RMers 27UCDecision

基于 ROS 2 Humble、Nav2 和 BehaviorTree.CPP 的 **27 赛季 RMUC 决策系统**。

## RMUC 场地适配

- 战场基准：28m x 15m，使用官方俯视图左下角作为场地坐标原点，红方向蓝方为 `+x`。
- 蓝方适配：对红方语义点执行中心对称，无需维护第二套坐标。
- 地图校准：使用 `field_to_map` 将官方场地坐标变换到机器人 SLAM 的 `map` 坐标。
- 行为树只引用 `supply`、`center_lower` 等语义名称，不再硬编码坐标。

主要文件：

- `rm_behavior_tree/config/rmuc_2026_field.yaml`：尺寸、阵营、变换和语义航点。
- `rm_behavior_tree/config/rmuc_2026_strategy.yaml`：稳定 V1 的预设、巡航拓扑和安全阈值。
- `rm_behavior_tree/config/rmuc_2026.xml`：当前可运行的 RMUC 行为树。
- `rm_behavior_tree/maps/rmuc_2026/README.md`：现场建图及标定流程。
- `docs/stable_bt_v1.md`：UC 多预设稳定决策树设计。

YAML 内置航点来自官方图纸的名义位置，仅供仿真和初始调试。实车运行前必须在 RViz
中校准 `field_to_map`、逐点检查底盘净空，并将 `calibrated` 改为 `true`。

## 环境要求

- **操作系统**: Ubuntu 22.04 (Jammy Jellyfish)
- **ROS2 版本**: Humble Hawksbill
- **GCC 版本**: 11 或更高（支持 C++17）

## 编译

```bash
colcon build --symlink-install
```

## 启动

### 模拟裁判系统

```bash
chmod +x publish.sh
./publish.sh
```

### 决策

```bash
chmod +x run.sh
./run.sh
```

默认以红方启动。蓝方启动：

```bash
RMUC_ALLIANCE=blue ./run.sh
```

现场坐标系原点不与官方场地坐标重合时：

```bash
RMUC_FIELD_ORIGIN_X=1.20 \
RMUC_FIELD_ORIGIN_Y=-0.35 \
RMUC_FIELD_ORIGIN_YAW=0.012 \
./run.sh
```

### 杀死进程

```bash
lsof -i :1667
kill -9
```

## Maintainer

[WHUT-RMers](https://github.com/WHUT-RMers) — 武汉理工大学 RoboMaster 战队
