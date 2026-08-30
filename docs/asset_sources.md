# 模型与协议资源选择

## 已集成资源

| 资源 | 用途 | 选择原因 | 许可证/边界 |
|---|---|---|---|
| [Universal Robots ROS 2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) | UR5e 网格、惯性、关节和 Xacro | 原厂维护，ROS 2 Jazzy 分支，避免自画六轴臂造成运动学错误 | BSD-3-Clause；UR 网格按仓库说明使用 |
| [PickNik ROS 2 Robotiq Gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper) | Robotiq 2F-85 描述 | 含联动关节、网格和 ros2_control 示例 | BSD-3-Clause；发布前保留版权声明 |
| [OPC UA for Machine Tools](https://reference.opcfoundation.org/MachineTool/v102/docs/1.1) | 真实机床语义接口参考 | 定义机床身份、监控和生产相关信息模型 | 本项目只参考状态语义，不复制规范正文 |
| [MTConnect 2.5](https://www.mtconnect.org/standard-download20181) | 设备状态与事件命名参考 | 适合建立控制器无关的设备数据层 | 真实接入时按厂商适配器能力取舍 |

二进制依赖默认用 `ros-jazzy-ur-description` 和
`ros-jazzy-robotiq-description`；`factory.repos` 只用于需要调试上游源码时。

## 自建 CNC 的原因

网上的 CAD/网格常见问题是许可证不清、面数过高、只有视觉没有碰撞、内部尺寸不适合
UR5e 伸入。当前 `cnc_machine` 因而采用“自建可交互骨架 + 合法上游机器人网格”的方式：

- 前方是真实开口，门关闭时由门碰撞体封闭；
- 门是有限位、阻尼和位置控制器的棱柱关节；
- 腔内有切屑盘、T 槽台、夹具、主轴头、刀柄和刀具碰撞体；
- HMI、急停、启动按钮、三色灯只做视觉表达，不伪装成安全控制；
- 后续可替换外壳视觉网格，但保留低面数碰撞骨架和接口坐标系。

## 后续资源引入准则

1. 只接收许可证明确、来源可追溯的模型。
2. 视觉网格与碰撞网格分离；碰撞体优先用凸包或简单几何。
3. 导入前统一米、右手坐标系、关节零位和网格原点。
4. 每个资源在本文件记录来源、版本、许可证和改动。
5. 先验证可规划、可碰撞、可重复，再追求录屏画质。
