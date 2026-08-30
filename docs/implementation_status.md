# 实现与验收状态（2026-08-10）

## 已进入唯一生产运行路径

- ROS 2 Jazzy、Gazebo Harmonic/DART、Nav2、MoveIt 2、ros2_control；
- 两驱动轮 + 两被动万向支撑轮的差速底盘，外部滚轮/履带为外观，不宣称真实履带接触；
- 三台同构 CNC 的门、夹具、加工和故障状态机；
- AprilTag 工位对位、低位 RGB-D 无固定槽位原料、高位 RGB-D CNC 工件、成品框空槽检测；
- 双指等力闭合、同一实体双侧触觉、双轨总闭合量保持、30 mm 无辅助试举、支撑接触放置；
- 低电量充电、装料前故障改派、加工中困料隔离、库存和零件唯一归属；
- `execute_order.xml`、`load_raw.xml`、`unload_finished.xml` 三层 BehaviorTree.CPP 总控；
- 持久化 PLC 事件队列、任务去重、失败停止与人工对账；
- 受约束 Agent/MCP 高层入口，不能发布底盘或关节命令。

## 物理真实性边界

生产操作链不读取 Gazebo 工件真值，不写 entity pose，不使用夹爪 DetachableJoint，也不在
感知缺失时回退到固定坐标。PICK 必须由双指接触、关节保持和无辅助试举证明；PLACE 必须
由指定支撑面接触证明。MoveIt `AttachedCollisionObject` 仅是碰撞模型。

原料/CNC 夹具约束仅模拟工装：双指接管后才释放，工件获得支撑后才锁紧。它不负责运输，
也不会移动工件。真值观察器仅供测试布置和黑盒评分，且有自动化边界测试阻止其接回生产栈。

## 最近一次关键验收

- 完整 `./scripts/run_checks.sh` 通过；精确 Python 口径为 336 core、55 Agent、22 perception，
  共 413 项，并通过 C++ BehaviorTree 协议测试；
- 30 个固定物理 seed 中 29 个端到端完成，正式成功率 96.67%，通过 24/30 门槛；
- Dock 137/137、PICK 68/69、PLACE 68/68、Undock 136/136；
- 34 个 RGB-D 目标覆盖 20 个 50 mm 网格，X/Y 跨度 0.151/0.522 m；
- DeepSeek-v4-flash 固定集工具策略与 Schema 合规 15/15，含 2 个低层越权拒绝；
- 完整证据、哈希和唯一失败分析见 `docs/evaluation_results.md`。

正式 seed 墙钟中位数为 213.461 秒、P95 为 416.490 秒。该时间受云端 Gazebo 实时率影响，
不能直接等同于实机节拍。

## 仍未宣称完成

- 不宣称通用 6D 抓取、真实履带、实机部署或工业级可靠性；
- 无固定槽位料框仍仅支持同类直立圆柱、单层分离和有限遮挡，不是堆叠、侧躺、任意姿态的
  通用 6D bin-picking；RGB-D 当前估计圆柱三维中心，抓取姿态由直立圆柱类别先验生成；
- CNC、托盘和 Tag 关系经过示教/测量，工位搬迁需要重新标定；
- 没有真实 PLC、BMS、安全 PLC、STO 和风险认证，因此只能写 Gazebo 物理仿真系统；
- Agent/MCP 是上层交互，不会替代确定性调度、行为树、感知证据或运动控制。

## 推荐验证顺序

```bash
./scripts/run_checks.sh
./scripts/test_task_bt_protocol.sh
./scripts/test_gazebo_smoke.sh
./scripts/test_moveit_smoke.sh
./scripts/test_navigation_truth.sh
./scripts/test_docking_truth.sh
./scripts/test_sparse_bin_pick_truth.sh
./scripts/test_raw_to_finished_truth.sh
./scripts/test_charging_truth.sh
HEADLESS=false ./scripts/test_physical_order_truth.sh
./scripts/run_physical_seed_campaign.sh --profile production --seeds 101-130
```

`*_truth.sh` 中的 truth 表示外部黑盒验收，不表示运行时控制依赖真值。
