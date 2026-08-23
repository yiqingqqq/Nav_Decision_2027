# RMUC 2026 稳定预设决策树 V1

## 1. 定位

V1 复用旧 UL 决策的“多预设”形式，但不复用 UL 坐标。所有策略只引用 UC
语义点名，点位坐标统一由 `rmuc_2026_field.yaml` 提供。

当前仓库没有实际 Nav2 栅格地图，`field_map.calibrated` 也是 `false`。因此，
现有坐标只用于逻辑测试和仿真，不得直接用于实车导航。

## 2. 固定优先级

```text
比赛未运行 / 状态失效 / 定位失效
                    ↓
                  HOLD

低血量 ───────────→ SUPPLY
近期受击 ─────────→ FALLBACK
导航连续失败 ─────→ FALLBACK
其他情况 ─────────→ 当前预设 PATROL
```

优先级从高到低：

1. 停车和数据安全；
2. 低血量补给；
3. 受击撤退；
4. 导航失败恢复；
5. 预设巡航。

恢复规则使用双阈值迟滞：HP 低于 `hp_retreat` 进入补给，恢复到
`hp_recover` 后才退出，避免在阈值附近反复切换。

## 3. 三套预设

### defense

```text
home_defense → outpost_defense → home_defense
```

- 主要任务：基地和前哨侧防守；
- 受击目标：`home_defense`；
- 不进入 `attack_staging`；
- 作为首次 Nav2 和实车联调预设。

### balanced

```text
patrol_lower → center_lower → patrol_upper → center_upper → patrol_lower
```

- 主要任务：己方巡航点与中场控制点之间轮巡；
- 受击目标：`fallback`；
- 默认预设；
- 在没有动态敌方信息时，不继续深入进攻区。

### aggressive

```text
center_lower → attack_staging → center_upper → center_lower
```

- 主要任务：中场到进攻准备区的前压；
- 受击目标：`fallback`；
- 只有地图、路径和底盘验证完成后才能启用。

## 4. 巡航执行规则

- 路线按固定顺序循环，禁止随机偏移目标点；
- 到点判定同时考虑 Nav2 成功结果和距离阈值；
- 每个目标最多重试 `navigation_retries` 次；
- 重试前必须取消旧目标；
- 连续失败后转向 `fallback_waypoint`；
- fallback 也不可达时进入 HOLD，不继续发送新目标；
- 任务最短保持时间和到点停留时间后续通过参数加入；
- 红蓝方只转换语义点位，不复制两套行为树。

## 5. UC 点位状态

当前逻辑使用以下九个名义点：

| 点位 | 用途 | 当前状态 |
|---|---|---|
| `supply` | 补给 | 待实测 |
| `home_defense` | 基地防守/安全驻留 | 待实测 |
| `outpost_defense` | 前哨防守 | 待实测 |
| `patrol_lower` | 下侧巡航 | 待实测 |
| `patrol_upper` | 上侧巡航 | 待实测 |
| `center_lower` | 下侧中场控制 | 待实测 |
| `center_upper` | 上侧中场控制 | 待实测 |
| `attack_staging` | 进攻准备 | 待实测 |
| `fallback` | 受击撤退 | 待实测 |

逻辑点位不等于最终导航点。建图后，每个区域应至少增加：

- 入口点；
- 驻留点；
- 退出点；
- 不可达时的备用点。

## 6. 实现边界

V1 暂不包括：

- Utility AI 动态任务评分；
- 敌方威胁地图；
- 多车协同占位；
- 自动选择交战点；
- 从官方渲染图直接生成占用栅格。

V1 完成后，行为树只负责可靠执行。后续世界模型和 Utility AI 可以动态选择
预设、区域或任务，不需要重写导航和安全子树。

## 7. 进入实车前的硬条件

- 存在实际 `occupancy.pgm` 和 `occupancy.yaml`；
- 完成 `field_to_map` 标定并保存残差；
- 所有启用点位通过 footprint 净空检查；
- 所有相邻巡航点之间通过 Nav2 可达性检查；
- 红蓝两方均完成仿真或低速 dry-run；
- 将 `field_map.calibrated` 设为 `true`；
- 实车模式在 `calibrated=false` 时必须拒绝发送导航目标。
