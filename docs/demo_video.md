# 无界面演示视频与证据链

## 演示目标

视频不强行限制为三分钟。主片只保留能证明系统闭环的内容：

1. 操作员输入自然语言生产要求；
2. LLM 选择受限 MCP 工具并读取工厂状态；
3. AgentCommand 通过 Schema 和工具白名单；
4. 订单进入 BehaviorTree.CPP；
5. Nav2 到原料区、AprilTag 精对位、RGB-D 选件、MoveIt 双指物理抓取；
6. CNC 开门、装夹、关门、加工、卸料；
7. 成品区深度占用检测、支撑接触释放；
8. 最终库存、机床、夹爪和订单状态一致。

补充短片分别展示低电量自动充电和故障机床改派。不要把多个重复导航过程
完整保留；可以用 2 倍速，但不得剪掉失败、人工干预或伪造成功证据。

## 云端为何不需要 GUI

`factory.sdf` 中有一个无碰撞的固定观察相机。默认不激活，普通统计不会承担
720p 渲染开销。录像脚本单独启动 `ros_gz_bridge` 订阅后，相机才由
Gazebo 的 `--headless-rendering` 路径离屏渲染。ROS 图像由
`factory_perception record_overview` 原子写入 MP4；任务结束后再使用
FFmpeg/NVENC 生成 2 倍速成片。

运行：

```bash
export OPENAI_API_KEY=...
export OPENAI_BASE_URL=https://api.deepseek.com
export FACTORY_AGENT_MODEL=deepseek-v4-flash

tmux new-session -s agent_demo
./scripts/record_agent_factory_demo.sh \
  artifacts/demo/agent_factory_cycle \
  "请先查询工厂状态，然后加工1个原料并放入成品区。"
```

API Key 只来自进程环境，不写入仓库、视频或日志。

## 产物

成功运行后目录必须同时包含：

- `operator_prompt.txt`：原始输入；
- `agent_trace.json`：模型回复、工具调用和工具结果；
- `agent_audit.jsonl`：真实 ROS Agent 的追加式命令审计；
- `physical_stack.log`：BT、导航、感知、MoveIt、PLC 状态；
- `final_factory_state.json`：ROS 最终状态；
- `factory_overview_raw.mp4`：原速离屏画面；
- `agent_factory_cycle_2x.mp4`：2 倍速物理主片；
- `agent_factory_cycle_presented.mp4`：包含 Agent 请求和终态说明的交付版；
- `video_probe.json`：时长和文件大小；
- `SHA256SUMS`：两版视频、Agent trace、审计和最终状态哈希。

只有视频、Agent trace、最终 ROS 状态三者一致时，演示才算成立。单独一段动画
不能证明 Agent 调用了真实接口；单独一段工具日志也不能证明机器人完成了物理任务。

## 同步到本机

```bash
rsync -az ucloud-robot:/home/ubuntu/Embodied_Robotic_Arm/artifacts/demo/ \
  /home/taoxu/Embodied_Robotic_Arm/artifacts/ucloud_demo/
```

本机可直接播放 MP4；无需 Gazebo GUI。若需要剪辑，只从原始 MP4 和保存的时间戳
派生新文件，保留原始证据不覆盖。

## 已完成的代表性录像

2026-08-10 云端 RTX 4090 无界面运行已完成。自然语言请求经过真实
DeepSeek-v4-flash、MCP 和 ROS 2 Agent 后，仅执行一个状态变更工具
`submit_order`；BT 完成一个工件的全流程。原速录像 151.70 s，2 倍速主片
75.85 s，带说明的交付版 83.85 s。最终状态为毛坯 3、成品 1、无持件、无活动订单。

本机证据目录：
`artifacts/ucloud_20260810/demo/agent_factory_cycle/`。交付版为
`agent_factory_cycle_presented.mp4`，其 SHA-256 为
`ed5ae73db241dcfa27e977d862e735f6692ea73beabe3d90b57c713bc7d7193c`。

## 最终综合演示

最终综合片由四段通过独立验收的证据组成：

1. 真实 DeepSeek 请求经 MCP 和 ROS2 提交单件生产订单，并完成随机料位 RGB-D 选件；
2. 两件订单先装入两台 CNC，机床并行加工后按完成事件依次回收；
3. 20% 初始电量触发视觉充电对接，达到恢复阈值后继续原订单；
4. 首选 `machine_2` 装料前故障，调度器改派 `machine_1` 并完成订单。

后三段使用固定工装布局，分别隔离验证调度、能源恢复和故障恢复；随机工件鲁棒性由
第一段真实 Agent 录像和 30-seed 物理回归单独证明。每段只有 Action 成功、库存终态
一致且原始 MP4 完整时才允许进入合成片。重复导航采用 6 倍速，物理动作和失败判定未
通过物体位姿传送或剪辑伪造。

云端可恢复录制命令：

```bash
tmux new-session -s final_showcase
./scripts/cloud_record_final_showcase.sh artifacts/ucloud_final_showcase
```

最终视频时长 193.50 s，文件为
`artifacts/ucloud_20260810/final_showcase/ros2_mobile_manipulator_final_showcase.mp4`，
SHA-256 为
`70ed805ec6ab0d580369a71142ef5263101f649391a1d9387b2e335e2a778694`。
目录同时保留各章节原始视频、倍速视频、Action 结果、工厂终态、日志和哈希。
