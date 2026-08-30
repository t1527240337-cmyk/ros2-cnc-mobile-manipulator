# 正式评测结果与统计口径

本文是项目量化结果的唯一入口。日期为 2026-08-10；除非新的完整固定种子批次覆盖本文，
其他文档不得用少量定向回归替换这里的数字。

## 1. 物理端到端评测

运行环境：Ubuntu 24.04.4 LTS、ROS 2 Jazzy、Gazebo Harmonic/DART、16 vCPU
AMD EPYC 7413、31 GiB 内存、RTX 4090 24 GB。命令为：

```bash
./scripts/run_physical_seed_campaign.sh --profile production --seeds 101-130
```

30 个种子都从新进程和新场景开始；失败不会停止后续 seed，也不会自动重跑。场景覆盖
3–6 个原料、每 6 个 seed 一次两件订单、每 5 个 seed 一次 20% 低电量回充、每 7 个
seed 一次首选 CNC 故障改派，以及底盘初始位姿扰动。

| 指标 | 实测结果 |
|---|---:|
| 端到端完成 | 29/30 |
| 正式成功率 | 96.67% |
| 既定门槛 | 24/30，已通过 |
| RGB-D 实际选件样本 | 34 |
| 50 mm 空间网格 | 20 |
| X/Y 覆盖跨度 | 0.151 / 0.522 m |
| seed 墙钟中位数 | 213.461 s |
| seed 墙钟 P95 | 416.490 s |
| 成功 seed 墙钟中位数 | 215.105 s |

完整结果位于
`artifacts/ucloud_20260810/physical_seed_campaign_v47_final_production/`。
`summary.json` 和 `bt_leaf_metrics.json` 分别有 SHA-256
`adbe5896d59fd26dbc5a31db6b4e10126ed40642881c492a18b6e1cb2c8b749a` 与
`dec41256b89934eecd749085d6d94ddf51380eb3905eee41e49c1812223d9428`。

唯一失败是 seed 124：控制器状态表明五个关节到位，`arm_wrist_1_joint` 在距目标
0.261 rad 处受阻；结合录制画面与无序原料布局，定位为横向预抓取走廊穿过邻近工件，
而不是规划超时不足。修复将原料抓取改为目标正上方垂直下探，并按实际 Tag—料台刚性
偏移修正 Planning Scene。独立复测中 seed 124 完整通过。原 30-seed 结果仍保留为
29/30，未用修复后的定向复测回填正式统计。

修复后另运行 production seed 120–126：首次为 5/7，seed 124 已通过；seed 120 在第二件
成品放置时检测到已放工件与腕部的规划碰撞，seed 121 被 Nav2 判定起点占用，系统均拒绝
继续执行。两者从全新进程单独复跑均完整通过（2/2），说明它们是非确定性状态采样问题，
不是固定场景必现失败。该小样本只作为故障分类和回归证据，不替代正式 30-seed 指标。

## 2. BehaviorTree.CPP 叶节点

统计直接解析 30 个 seed 的 BT 日志；一次订单可多次执行同名叶节点，因此这里报告动作
尝试而不是 seed 数。

| 叶节点 | 成功/尝试 | 成功率 | 中位耗时 | P95 |
|---|---:|---:|---:|---:|
| DockStation | 137/137 | 100% | 20.282 s | 31.970 s |
| PickPart | 68/69 | 98.55% | 20.162 s | 22.653 s |
| PlacePart | 68/68 | 100% | 25.213 s | 28.718 s |
| UndockStation | 136/136 | 100% | 1.843 s | 2.046 s |
| EnsureEnergy | 68/68 | 100% | 0.021 s | 37.102 s |
| Open/CloseMachineDoor | 68/68 | 100% | 0.061 s | 0.081 s |
| WaitMachineDone | 34/34 | 100% | 11.980 s | 12.189 s |

库存提交、CNC 装卸确认、加工启动和任务终态叶节点均为 100%。这些数字不等同于实机
可靠性，只说明固定仿真分布中的真实运行路径。

## 3. Agent 与 MCP 评测

DeepSeek-v4-flash 使用真实模型响应和 MCP 服务器公开的真实 JSON Schema，工具返回值为
隔离的合成工厂快照，避免语言评测改变正在运行的物理批次。15 条固定中文用例全部通过：

- 5 类只读查询；
- 8 类下单、自动模式、暂停/恢复/取消和机床保持等受限高层操作；
- 2 类低层运动/虚假急停请求均未产生工具调用。

因此可写成“固定集工具策略与 Schema 合规率 15/15（100%）”，不能写成“物理 Agent
端到端成功率 100%”。原始结果位于
`artifacts/ucloud_20260810/agent_eval/deepseek_v4_flash_15_cases.json`。

另完成 1 次真实 Agent 到物理仿真的代表性闭环：DeepSeek 先查询工厂状态与能力，只调用
一次 `submit_order(quantity=1)`，随后只读查询任务；BehaviorTree.CPP 实际完成取料、
CNC 装料/加工/卸料和成品放置。最终毛坯 4→3、成品 0→1、夹爪无持件、订单清空，
交付版视频 83.85 s。该单次演示用于证明接口链路，不并入 15 条语言合规率或
30-seed 物理成功率。

## 4. 自动化测试与复现

本机完整 `./scripts/run_checks.sh` 通过。精确 Python 测试口径为 336 项 core、
55 项 Agent、22 项 perception，共 413 项；另有 C++ BehaviorTree 协议测试、Gazebo
黑盒验收和 ROS 接口 smoke tests。Docker 构建与容器内全检查的结果保存在
`artifacts/ucloud_20260810/docker_validation/`。

## 5. 仿真边界

- 生产链不读取 Gazebo 工件真值、不写 entity pose、不使用运输附着关节；
- 原料是同类直立圆柱、单层分离、无固定槽位；RGB-D 估计中心，姿态来自类别先验；
- 成品区是四个教导区域，RGB-D 连续观测决定区域是否可用；
- 底盘是两驱动轮加两个被动支撑轮，履带仅为外观；
- CNC、Tag 与夹具关系经过示教/标定，工位搬迁需要重新标定；
- 没有真实 PLC、BMS、安全 PLC、STO 或风险认证，不宣称实机或工业等级。

GPU 主要承担离屏相机渲染和 NVENC；Gazebo/DART、MoveIt 和 Nav2 的主要瓶颈仍是 CPU。
