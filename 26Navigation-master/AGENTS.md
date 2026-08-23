# 导航与隧道模式的适配

## 一、当前实现存在一个顺序上的根本问题

把“是否走隧道”放到全局路径生成之前，而不是路径生成之后。

## 二、确认 0.32 m 是机器人半径

## 三、参数调整

  - 普通 TEB 的 min_obstacle_dist: 0.35
  - 普通 TEB 的 inflation_dist: 0.35
  - 普通 TEB 的 weight_obstacle: 50
  - 全局规划每秒重新规划，但需要有对隧道路径的保持


  ### 隧道控制器配置不完整

  当前 TunnelController 只明确设置了速度、footprint 和少量障碍参数：

  26Navigation-master/src/rm_nav_bringup/config/simulation/nav2_params_sim.yaml:258

  许多对狭窄通道很重要的参数仍使用 TEB 默认值，例如：

  - 全局路径中间点提取
  - 中间点顺序约束
  - weight_viapoint
  - 优化迭代次数
  - 前瞻距离
  - 是否允许倒车初始化
  - 是否覆盖路径姿态
  - 同伦类别切换
  - 隧道专用 progress checker 和 goal checker

  因此它目前更像“低速版 TEB”，还不是完整的“隧道控制器”。

  ———

  ## 四、姿态需要专门处理

  对于红方这条路线修改为：

  (5.0, -3.0)
  → (4, -3.6)
  → (2.5, -3.6)
  → (0.5, -3.6)


  1. 到达 (4.0, -3.6) 附近的入口准备点。
  2. 在隧道外把朝向调整到约 π。
  3. 朝向满足误差后，进入“已承诺通过隧道”状态。
  4. 沿有序中心线点通过，不再重新选择大路。
  5. 完全驶出出口后才恢复普通规划。

  反向通过时自动交换入口和出口，隧道方向约为 0。

  建议隧道入口/内部使用独立姿态规则：

  - yaw 容差约 0.1～0.25 rad
  - 禁止倒车初始化
  - 有序 via points
  - 入口前软性对准，进入后强制保持隧道方向
  - 隧道内禁用旋转恢复；失败时沿中心线直退到入口准备点

  如果机器人 footprint 是圆形，朝向不影响几何碰撞半径，但仍影响 TEB 对前进、横移、旋转速度的分配。如果实际车体是矩形，用 polygon footprint 后，朝
  向会更加重要。

  Nav2 也提供 Rotation Shim，用来先转到路径方向再交给主控制器；它支持全向底盘。Nav2 Rotation Shim 文档
  (https://docs.nav2.org/tutorials/docs/using_shim_controller.html)

  ———
 ## 五、

  ### 首选新优化器：MPPI（暂时不用）

  Nav2 的 MPPI 是采样式预测控制器，支持全向、差速和 Ackermann 模型，可通过多个 critic 同时考虑：

  - footprint 碰撞
  - 全局路径贴合
  - 路径朝向
  - 障碍代价
  - 速度和运动约束

  这比当前 TEB 在狭窄空间中的单次非线性局部优化更容易跳出局部坏解，也能直接配置 Omni 模型。Nav2 MPPI 官方文档
  (https://docs.nav2.org/configuration/packages/configuring-mppic.html)

  但 MPPI 同样不能解决“全局路线根本没走隧道”。它需要配合明确的隧道中心线或强制路径。

  ### 中心线跟踪器

  隧道基本是直线地形时，未必需要复杂优化器。固定中心线之后，一个确定性的路径跟踪器可能比 TEB 更稳定：

  入口外对准
  → 沿隧道中心线低速跟踪
  → 碰撞预测
  → 出口解除

  RPP 擅长精确路径跟踪和近障减速，但官方主要将其定位于差速、Ackermann
  等平台；全向车可以在隧道阶段暂时按“只前进+旋转”使用，但不会发挥横移能力。Nav2 RPP 文档
  (https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)


  ## 六、
  ### 构建语义路线图

  把场地抽象为：

  场地节点 ──普通道路边── 中部节点
       └────隧道边──────┘

  决策层选择“隧道边”后，导航层只负责沿这条边生成和跟踪路径。入口不是固定方向，而是根据当前位置自动选择边的起点。

  这是我认为最适合比赛场地的长期架构：场地固定、特殊地形有限，没必要让纯栅格规划器每次重新猜拓扑。

  ### 建立代价地图语义掩膜

  Nav2 支持 preferred lane / keepout costmap filter，可以：

  - 隧道任务中提高大路代价；
  - 保持隧道中心线为低代价区域；
  - 对隧道设置限速区域；
  - 普通任务关闭该掩膜。

  Nav2 官方将 costmap filter 用于禁行区、偏好车道和限速区。Costmap Filter 官方说明
  (https://docs.nav2.org/configuration/packages/map_server/configuring-costmap-filter-info-server.html)、Preferred Lanes 教程
  (https://docs.nav2.org/tutorials/docs/navigation2_with_keepout_filter.html)

  它比简单降低全局 inflation 更合理，因为降低 inflation 会同时影响全场安全性，而语义掩膜只影响隧道任务或指定区域。


