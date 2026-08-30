# 验收和基准测试

## 立即可运行的语义基准

```bash
PYTHONPATH=src/factory_core python3 -m factory_core.benchmark --trials 30 --output benchmark_results.csv
```

该基准随机化加工周期、初始电量，并以 25% 概率在订单开始前故障一台空机床，验证剩余机床是否可以继续完成订单。

## 物理仿真验收

固定保存 30 个场景种子。每个种子确定性生成安全范围内的机器人初始位姿扰动和托盘零件位置，并轮换机床；特定种子覆盖低电量与空机故障。当前没有按种子改变传感器噪声或障碍物，因此不把这两项写入统计范围。记录：

- 完整订单成功率，目标至少 24/30；
- Nav2 到达和视觉停靠成功率，分别至少 90%；
- 抓取、装夹、卸载和成品放置成功率，分别至少 90%；
- 停靠平移误差小于 3 cm、偏航误差小于 3°；
- LLM 断网和非法输出时，规则订单仍可完成；
- 订单结束后的库存、夹爪和机床零件 ID 完全一致。

任何对外发布的性能数据都必须来自该批量脚本的实际结果，不使用设计目标代替测量值。

## 固定种子 Gazebo 物理统计

非 Agent/MCP 的单件生产统计入口：

```bash
./scripts/run_physical_seed_campaign.sh \
  --profile production \
  --seeds 101-130
```

每个种子都执行真实 `ExecuteOrder`，并固定启用无序单层原料 RGB-D 选择、双指接触、
无辅助证明抬升、CNC 门/夹具握手、成品框可观察空槽选择和最终库存核对。场景表由
种子确定而不是运行时暗中改变：

- 三台 CNC 按种子轮换；
- 原料数量在 3–6 件变化，整个工作区随机采样且任意两件中心距至少 130 mm；
- 每第 6 个种子执行两件订单，验证第一次抓取后重新感知并选择下一件；
- 初始底盘位姿在 X/Y 各 ±6 cm、偏航 ±0.048 rad 内确定性扰动；
- 每第 5 个种子从 20% 电量开始，要求先回充再恢复订单；
- 每第 7 个种子在装料前故障首选 CNC，要求改派下一台健康机床。

结果逐种子保存在
`artifacts/ucloud_20260810/physical_seed_campaign_v47_final_production/seed_<N>/result.json`，
同目录保留控制台、Action、工厂状态、机床状态和完整物理栈日志。场景 JSON 与每个证据
文件都记录大小和 SHA-256；恢复时拒绝场景变化、文件缺失或日志篡改。失败默认是统计终态，
只有明确使用 `--rerun-failed` 才会覆盖，因此正式档不会通过隐式重试美化数据。

2026-08-10 的完整 101–130 批次形成 30 个终态，29/30 通过（96.67%），
`threshold_met=true`、`qualification_met=true`。34 个 RGB-D 抓取目标覆盖 20 个
50 mm 网格，X/Y 跨度 0.151/0.522 m。DockStation 137/137、PickPart 68/69、
PlacePart 68/68、UndockStation 136/136。

唯一失败 seed 124 保留在正式档：五个关节到位，`arm_wrist_1_joint` 在距目标
0.261 rad 处受阻；结合录像和场景布局，根因是横向预抓取走廊穿过邻近工件。通用修复
改为目标正上方垂直下探并校正 Planning Scene 的 Tag—料台偏移，seed 124 定向复测通过；
正式 30-seed 数字仍不回填。完整指标、哈希和仿真边界见
`docs/evaluation_results.md`。只核验现有证据时使用同一命令并加
`--verify-only`。

三件三机床的长流程可以使用独立档案目录：

```bash
./scripts/run_physical_seed_campaign.sh \
  --profile three-machine \
  --seeds 201-203
```

该档每个种子都是三工件整单，耗时显著更长，不与单件统计混合。

## 成品框空槽验收

较短的机械可达性验收：

```bash
HEADLESS=false ./scripts/test_finished_slot4_truth.sh
```

它从固定原料槽 2 执行真实双指抓取和无辅助试举，经工厂路线到成品框，再把工件放入
冗余槽位 4。2026-07-30 已通过一次：路线、两次视觉对位、物理放置和离站全部成功，
放置水平误差约 1 mm。

三零件感知选槽整单验收：

```bash
HEADLESS=false ./scripts/test_finished_slot_perception_truth.sh
```

该脚本启用 RGB-D 槽位选择，要求每次成品放置前连续三帧确认可观察且为空，并检查三次
选择互不重复且每次成功放置后都记录槽位保留。四个候选中允许一个被机械结构遮挡，
未知槽位不能当作空槽。2026-07-30 的无界面正式验收已通过：60/60 步、三件物料分别
经过 machine_1/2/3 加工，成品依次放入 2、1、4 号槽，最终原料/成品库存为 3/3，
订单用时 1407.0 秒、脚本墙钟 1411 秒。这是一次完整通过记录，不能表述为统计成功率。

## 加工中故障隔离验收

```bash
HEADLESS=false ./scripts/test_loaded_machine_fault_truth.sh
```

该脚本先把两个不同工件物理装入 `machine_2` 和 `machine_1`，只在 `machine_2` 的 PLC
状态真正变为 `PROCESSING` 后注入故障。验收要求机械臂不得打开故障机或重放其卸料，
但必须继续回收健康的 `machine_1`；最终还要同时核对 Action 错误码、库存、夹爪、
故障机零件 ID 和健康机状态。

2026-07-30 的无界面正式验收已通过：`machine_2` 保持 `FAULT/code 99`，其夹具保留
`raw_part_2`；`machine_1` 的 `raw_part_1` 完成物理卸料并进入成品框。最终原料库存
为 4、成品库存为 1、夹爪为空，Action 返回 `completed=1/error_code=21` 和明确的
人工介入说明，脚本墙钟 720 秒。这里的 `success=false` 是预期的安全业务结果。

## 任务队列分层验收

不启动 Gazebo 的快速协议验收：

```bash
./scripts/test_machine_task_queue_ros.sh
./scripts/test_machine_task_worker_ros.sh
FAIL_FIRST=true ./scripts/test_machine_task_worker_ros.sh
./scripts/test_machine_task_recovery_ros.sh
```

第一条验证机床快照生成的任务可以持久化且重启不重复；第二条使用隔离 ROS 域和假物理
Action Server，验证 `PENDING → RESERVED → RUNNING → SUCCEEDED` 以及串行派发。
第二条还故意启用 `use_sim_time` 却不发布 `/clock`，以证明派发心跳不会被仿真时钟
切换卡死。第三条让第一项物理任务返回不可重试失败，要求 Worker 停止继续派发，并验证
其余两项仍为 `PENDING`，等待机器人、工件、库存和 PLC 人工对账。

第四条把首项任务停在 `physical_transfer` 反馈阶段并杀死 Worker。重启后要求原
`RUNNING` 任务成为 `FAILED`、最后阶段仍能查询、其余任务不派发；未声明完成现场核对
的重试请求必须被拒绝。随后测试以操作员身份确认“夹爪为空且工件仍在原槽位”，通过
`ReconcileRobotTask.srv` 授权同一 `task_id` 重试，最终原有三项任务全部成功。
2026-07-30 的隔离 ROS 验收已通过：
`checkpoint=physical_transfer/rejected_unverified=true/retried=同一任务/succeeded=3`。

真实 Gazebo 六任务闭环：

```bash
HEADLESS=false ./scripts/test_machine_task_worker_truth.sh
```

它不会提交 `ExecuteOrder`，而是让三台机床的 IDLE/DONE 事件依次驱动三次上料和三次
卸料，并检查六项任务、最终库存、夹爪寄存器和三台机床状态一致。

2026-07-30 的无界面正式验收已通过：执行顺序为装1→装2→卸1→卸2→装3→卸3，
6/6 任务成功，原料库存 6→3、成品库存 0→3，三台 CNC 最终均为 IDLE，物理抓取均
通过双指接触和无辅助证明抬升，脚本墙钟 1475 秒。
