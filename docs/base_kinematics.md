# 履带外观差速底盘物理模型

## 当前模型是什么

当前 Gazebo 验收主线不是连续履带接触，也不是四个刚性驱动轮同时接地，而是：

- 左右两个中部驱动轮承担牵引和差速转向；
- 前后各一个被动万向支撑轮承担俯仰支撑；
- 每侧另外两个外侧滚轮和履带带体只负责表达履带外观，不参与地面碰撞；
- `ros2_control` 同步转动六个可见滚轮，但物理驱动力只来自两个中部轮。

这相当于“履带外观 + 两轮差速物理底盘”。选择它是为了让平整工厂中的 Nav2、
Docking 和机械臂物理操作具有可复现的基线。项目文档不得把它描述成高保真履带
接触仿真。

## 为什么从四轮 skid-steer 改成这个方案

四个刚性轮或连续履带原地转向时必须产生可控的横向滑移。Gazebo 中若四个接触点的
横向约束过强，底盘容易前后摆动或编码器在转、本体却转不动；若只反复降低普通摩擦，
直行牵引和停止精度又会一起变差。

Gazebo 的 WheelSlip 和 TrackedVehicle 都可以继续研究，但它们会把时间投入到履带
接触参数辨识，而本项目的关键交付是移动操作闭环。当前方案显式承认这个工程取舍，
同时保留履带视觉结构，后续可以替换底盘物理层而不改 Nav2、Docking 或任务接口。

## 参数分工

- URDF/Xacro 定义驱动轮、万向支撑轮、质量、惯量和碰撞体；
- `diff_drive_controller` 负责左右轮速度与轮式里程计；
- 被动万向轮由 Gazebo 物理解算，不接收速度命令；
- 外侧滚轮跟随左右驱动速度做视觉动画，不提供额外牵引；
- `/ground_truth/odom` 来自 Gazebo 独立真值，用于发现轮式里程计与本体运动不一致。

配置位置：

- `src/mobile_manipulator_description/urdf/mobile_manipulator.urdf.xacro`
- `src/mobile_manipulator_description/config/controllers.yaml`
- `src/factory_bringup/launch/gazebo.launch.py`

## 自动验收

```bash
./scripts/test_base_kinematics.sh
./scripts/test_navigation_truth.sh
```

验收至少比较：

- 直行真值距离与轮式里程计；
- 原地转向真值角度与轮式里程计；
- 横向漂移；
- Nav2 到工位预停靠位的独立 Gazebo 真值误差。

这些是仿真回归数据，不是实机系统辨识结果。迁移真实履带底盘时，需要用编码器、
IMU、角速度和外部定位重新标定有效轮距、滑移模型与控制器增益。

## 后续何时值得切换真履带

只有当项目要研究坡面、越障、松软地面或履带滑移估计时，才值得把主线替换为
`gz::sim::systems::TrackedVehicle` 或更细的履带接触模型。对于平整机床车间，
当前差速物理底盘足以验证导航、视觉对位和移动操作系统架构。
