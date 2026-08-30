# Gazebo 场景对象与人工控制

## CNC 朝向

`cnc_machine` 的局部门面朝 `-X`。SDF 的姿态格式为：

```text
<pose>x y z roll pitch yaw</pose>
```

旋转只修改最后一个 `yaw`，单位为弧度：`90° = 1.5708`，`-90° = -1.5708`。
当前三个 CNC 位于工厂上方，门需要朝下方中央通道，因此使用 `yaw=1.5708`。

修改世界文件后必须重新构建并重启 Gazebo：

```bash
colcon build --symlink-install --packages-select mobile_manipulator_description
source install/setup.bash
./scripts/run_gazebo_demo.sh
```

## 开关 CNC 门

Gazebo 演示运行时，在另一个终端执行：

```bash
source scripts/setup_ros_env.sh
./scripts/set_cnc_door.sh machine_1 open
./scripts/set_cnc_door.sh machine_1 close
```

该命令经过 `Machine` 状态机校验。加工中、进给保持或故障互锁不满足时会被拒绝，
而不是直接绕过状态机向门电机发布位置。

底层调试时，门关节对应的 ROS 话题为 `/machine_1/door_position_cmd`，0.0 米为关闭，
1.18 米为打开；正常业务代码不应直接发布这个话题(现实中机械臂发个TCP应用层包让机床将门控寄存器/虚拟IO置为1/0就行)。

## 夹爪控制

仿真执行机构采用两个独立物理滑轨和等力控制。业务代码先以低力搜索接触，只有同一工件
产生新鲜双侧触觉后才切换到承载保持力；两指总行程表示开度，行程差表示工件偏心。手动
调试可使用：

```bash
./scripts/set_gripper.sh close
./scripts/set_gripper.sh open
./scripts/set_gripper.sh stop
```

该脚本直接面向调试用 effort controller；生产操作必须走 `ManipulatePart.action`，由双侧
触觉、关节状态和证明抬升共同决定成功，不能只凭一次夹爪命令提交库存变化。

## 三个简化场景对象

- 黄色柱体/方柱 `temporary_obstacle`：用于测试 Nav2 绕障和重规划，可删除或移动。
- 绿色容器 `finished_bin`：成品收货篮；蓝色容器 `raw_bin` 是毛坯篮。
- 橙色加绿色 `charge_dock`：橙色是充电座外壳，绿色是接触/对准区域。

它们采用低面数功能几何，但都已进入物理验收：原料台提供无固定槽位的单层随机工作区，
成品台由 RGB-D 选择空闲放置区域，充电座具有导向面与接触/电流证据。后续若提高视觉质量，
可以替换外观网格和增加护栏，但不能退回固定工件坐标或取消碰撞；黄色障碍物应保留，
因为它承担可复现的导航故障测试。
