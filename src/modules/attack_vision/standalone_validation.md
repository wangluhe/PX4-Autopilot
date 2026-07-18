# attack_vision 独立验证流程

本文档用于验证 `src/modules/attack_vision` 在不启动上位机时的独立工作能力，覆盖 SITL 仿真和实机测试。

## 1. 修改影响边界

当前新增的 `AV_SIM_*` 注入参数只在 `__PX4_POSIX` 编译路径生效，也就是只用于 SITL/POSIX 仿真：

```text
AV_SIM_PIX_EN
AV_SIM_PIX_X
AV_SIM_PIX_Y
AV_SIM_GMB_EN
AV_SIM_GMB_ROLL
AV_SIM_GMB_PIT
AV_SIM_GMB_YAW
```

在实机 NuttX 固件中，这些参数可以存在于参数表里，但不会覆盖真实吊舱串口帧，也不会驱动实机吊舱或飞控控制链路。

需要注意：以下改动是实机路径会使用的，属于实机功能的一部分：

```text
串口非阻塞读取和帧超时保护
attack_vision_status 调试话题
AV_PIX_X_INV / AV_PIX_Y_INV
AV_GMB_YAW_INV / AV_GMB_PIT_INV / AV_GMB_ROLL_INV
parameter_update 运行时参数刷新
```

也就是说，仿真注入不涉及实机操作；但安全性、可观测性和坐标符号参数会在实机中生效。

## 2. 通用观测指令

在 PX4 shell 或 QGC MAVLink Console 中使用：

```sh
attack_vision status
listener attack_vision_status
listener trajectory_setpoint
listener offboard_control_mode
listener vehicle_status
listener vehicle_attitude
```

常用参数查看：

```sh
param show AAATTKVIS_EN
param show AV_EXT_MODE
param show AV_FORWARD_V
param show AV_MAX_VZ
param show AV_PIX_X_INV
param show AV_PIX_Y_INV
param show AV_GMB_YAW_INV
param show AV_GMB_PIT_INV
param show AV_GMB_ROLL_INV
param show AV_SIM_PIX_EN
param show AV_SIM_GMB_EN
```

核心判断关系：

```text
pix_offset_x/y
  -> los_gimbal_x/y/z
  -> target_vec_ned_x/y/z
  -> trajectory_setpoint.velocity
```

其中：

```text
target_vec_ned_x = NED North 分量
target_vec_ned_y = NED East 分量
target_vec_ned_z = NED Down 分量

trajectory_setpoint.velocity ~= target_vec_ned * AV_FORWARD_V
velocity[2] 还会受 AV_MAX_VZ 限幅
```

如果目标在图像中心，吊舱相对机体角为 0，则：

```text
los_gimbal ~= [1, 0, 0]
target_vec_ned ~= 当前机体前向在 NED 下的方向
```

只有当机体 yaw 也接近 0 时，`target_vec_ned` 才接近 `[1, 0, 0]`。

## 3. SITL 仿真验证

### 3.1 编译

```sh
ninja -C build/px4_sitl_default
```

启动 PX4 SITL，使用工程已有的启动方式即可。本文只要求启动 PX4，不启动上位机。

### 3.2 基础参数

进入 `pxh>` 后设置：

```sh
param set AAATTKVIS_EN 1
param set AV_EXT_MODE 0
param set AV_FORWARD_V 0.2
param set AV_MAX_VZ 0.2

param set AV_PIX_X_INV 1
param set AV_PIX_Y_INV 0
param set AV_GMB_YAW_INV 0
param set AV_GMB_PIT_INV 0
param set AV_GMB_ROLL_INV 1

param set AV_SIM_PIX_EN 1
param set AV_SIM_GMB_EN 1
param set AV_SIM_PIX_X 0
param set AV_SIM_PIX_Y 0
param set AV_SIM_GMB_ROLL 0
param set AV_SIM_GMB_PIT 0
param set AV_SIM_GMB_YAW 0
```

如果模块已经在运行，重启模块：

```sh
attack_vision stop
attack_vision start
```

### 3.3 起飞并进入验证状态

可以通过 QGC 起飞，也可以用 PX4 shell：

```sh
commander arm
commander takeoff
```

等待飞机稳定后，观察：

```sh
listener vehicle_status
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
vehicle_status.nav_state: 14
attack_vision_status.lock_active: True
attack_vision_status.rc_offboard: True
attack_vision_status.allow_takeover: True
attack_vision_status.frame_valid: True
attack_vision_status.module_state: 2
```

### 3.4 像素脱靶量验证

中心：

```sh
param set AV_SIM_PIX_X 0
param set AV_SIM_PIX_Y 0
listener attack_vision_status
```

期望：

```text
pix_offset_x: 0
pix_offset_y: 0
los_gimbal_x ~= 1
los_gimbal_y ~= 0
los_gimbal_z ~= 0
```

画面右侧，实测吊舱输出 X 为负：

```sh
param set AV_SIM_PIX_X -200
param set AV_SIM_PIX_Y 0
listener attack_vision_status
```

期望：

```text
pix_offset_x: -200
los_gimbal_y > 0
```

画面左侧，实测吊舱输出 X 为正：

```sh
param set AV_SIM_PIX_X 200
param set AV_SIM_PIX_Y 0
listener attack_vision_status
```

期望：

```text
pix_offset_x: 200
los_gimbal_y < 0
```

画面下方，实测吊舱输出 Y 为正：

```sh
param set AV_SIM_PIX_X 0
param set AV_SIM_PIX_Y 200
listener attack_vision_status
```

期望：

```text
pix_offset_y: 200
los_gimbal_z > 0
```

画面上方，实测吊舱输出 Y 为负：

```sh
param set AV_SIM_PIX_X 0
param set AV_SIM_PIX_Y -200
listener attack_vision_status
```

期望：

```text
pix_offset_y: -200
los_gimbal_z < 0
```

### 3.5 吊舱角和 NED 速度链路验证

先回到中心：

```sh
param set AV_SIM_PIX_X 0
param set AV_SIM_PIX_Y 0
param set AV_SIM_GMB_ROLL 0
param set AV_SIM_GMB_PIT 0
param set AV_SIM_GMB_YAW 0
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
gimbal_roll_rad ~= 0
gimbal_pitch_rad ~= 0
gimbal_yaw_rad ~= 0
los_gimbal ~= [1, 0, 0]
target_vec_ned ~= 当前机体前向 NED
trajectory_setpoint.velocity ~= target_vec_ned * AV_FORWARD_V
```

吊舱向右 yaw +90 度：

```sh
param set AV_SIM_GMB_YAW 9000
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
gimbal_yaw_rad ~= +1.5708
target_vec_ned ~= 当前机体右向 NED
trajectory_setpoint.yaw ~= atan2(target_vec_ned_y, target_vec_ned_x)
trajectory_setpoint.velocity ~= target_vec_ned * AV_FORWARD_V
```

注意：`AV_SIM_GMB_YAW=9000` 表示吊舱永远固定在“相对机体右侧 90 度”。当前控制又会命令飞机 yaw 对准 `target_vec_ned`，因此长时间保持该注入值时，飞机可能会持续转圈。这是固定相对机体 yaw 注入与 yaw 控制闭环耦合导致的现象，不代表坐标转换错误。该项只做瞬时采样，采样后应恢复：

```sh
param set AV_SIM_GMB_YAW 0
```

吊舱上抬 pitch +10 度：

```sh
param set AV_SIM_GMB_YAW 0
param set AV_SIM_GMB_PIT 1000
listener attack_vision_status
```

期望：

```text
gimbal_pitch_rad > 0
target_vec_ned_z < 0
```

吊舱下俯 pitch -10 度：

```sh
param set AV_SIM_GMB_PIT -1000
listener attack_vision_status
```

期望：

```text
gimbal_pitch_rad < 0
target_vec_ned_z > 0
```

roll 需要配合非中心像素验证，因为中心视线 `[1, 0, 0]` 绕光轴滚转后方向不变：

```sh
param set AV_SIM_GMB_PIT 0
param set AV_SIM_PIX_X 0
param set AV_SIM_PIX_Y 200
param set AV_SIM_GMB_ROLL -1000
listener attack_vision_status
```

由于 `AV_GMB_ROLL_INV=1`，raw `-1000` 会转换成 `gimbal_roll_rad > 0`。

期望：

```text
pix_offset_y: 200
los_gimbal_z > 0
gimbal_roll_rad > 0
target_vec_ned_y 或 target_vec_ned_z 出现符合滚转方向的变化
```

### 3.6 Offboard 退出和恢复验证

从 Offboard 切到 Position：

```text
在 QGC 或遥控器中切 Position
```

期望：

```text
vehicle_status.nav_state 不再是 14
attack_vision 不继续发布有效末制导控制
飞机停止末制导，进入位置控制/悬停
```

再切回 Offboard：

```text
在 QGC 或遥控器中切 Offboard
```

期望：

```text
vehicle_status.nav_state: 14
attack_vision_status.module_state: 2
trajectory_setpoint 恢复按 target_vec_ned 发布速度
```

### 3.7 结束仿真注入

```sh
param set AV_SIM_PIX_EN 0
param set AV_SIM_GMB_EN 0
param set AV_SIM_PIX_X 0
param set AV_SIM_PIX_Y 0
param set AV_SIM_GMB_ROLL 0
param set AV_SIM_GMB_PIT 0
param set AV_SIM_GMB_YAW 0
```

## 4. 实机验证

### 4.1 实机安全前提

实机测试分两类：

```text
地面静态测试：不装桨或确保电机不会产生危险
低空动态测试：开阔场地、低速、小高度、随时可切 Position 接管
```

不要在首次验证时使用较大速度。推荐先使用：

```sh
param set AV_FORWARD_V 0.2
param set AV_MAX_VZ 0.2
```

### 4.2 实机参数

不启动上位机时，建议明确使用独立模式：

```sh
param set AAATTKVIS_EN 1
param set AV_EXT_MODE 0
param set AV_BAUD 115200

param set AV_PIX_X_INV 1
param set AV_PIX_Y_INV 0
param set AV_GMB_YAW_INV 0
param set AV_GMB_PIT_INV 0
param set AV_GMB_ROLL_INV 1

param set AV_FORWARD_V 1
param set AV_MAX_VZ 0.5
```

确认仿真注入关闭。实机中这些参数不会覆盖真实吊舱帧，但建议保持关闭，避免混淆记录：

```sh
param set AV_SIM_PIX_EN 0
param set AV_SIM_GMB_EN 0
```

### 4.3 起飞前静态测试

启动模块：

```sh
attack_vision stop
attack_vision start
attack_vision status
```

观察吊舱帧：

```sh
listener attack_vision_status
```

期望：

```text
frame_valid: True
frame_age_ms < 200
gimbal_*_raw_deg100 随吊舱运动变化
gimbal_*_rad 按 AV_GMB_*_INV 正确转换
pix_offset_x/y 随画面点击或锁定点变化
```

按已实测定义复查：

```text
画面左侧  -> pix_offset_x > 0
画面右侧  -> pix_offset_x < 0
画面上方  -> pix_offset_y < 0
画面下方  -> pix_offset_y > 0

吊舱右转  -> yaw 增大为正
吊舱左转  -> yaw 减小为负
吊舱上抬  -> pitch 增大为正
吊舱下俯  -> pitch 减小为负
光轴右滚  -> raw roll 为负，转换后 gimbal_roll_rad 为正
光轴左滚  -> raw roll 为正，转换后 gimbal_roll_rad 为负
```

如果未解锁，`target_vec_ned_valid` 可能不会持续更新，这是正常的；地面静态重点看帧、像素、吊舱角和参数转换。

### 4.4 串口安全验证

拔掉或停止吊舱数据源，观察：

```sh
listener attack_vision_status
attack_vision status
```

期望：

```text
模块不阻塞
QGC MAVLink Console 仍可操作
frame_valid 变为 False
frame_age_ms 增大
不再进入末制导控制
```

恢复吊舱数据后：

```text
frame_valid 重新变为 True
frame_age_ms 回到小于 200ms
```

### 4.5 起飞后动态测试

推荐流程：

1. 起飞到低高度并进入 Position，确认飞机悬停稳定。
2. 吊舱锁定目标，确认 `lock_active=True`、`frame_valid=True`。
3. 切入 Offboard 或 RC 末制导档位。
4. 观察 `attack_vision_status` 和 `trajectory_setpoint`。

观测：

```sh
listener vehicle_status
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
vehicle_status.nav_state: 14
attack_vision_status.allow_takeover: True
attack_vision_status.module_state: 2
trajectory_setpoint.velocity ~= target_vec_ned * AV_FORWARD_V
```

中心锁定时：

```text
los_gimbal ~= [1, 0, 0]
target_vec_ned ~= 吊舱光轴在 NED 下的方向
```

如果目标在画面右侧：

```text
pix_offset_x < 0
los_gimbal_y > 0
速度方向相对吊舱光轴向右修正
```

如果目标在画面下方：

```text
pix_offset_y > 0
los_gimbal_z > 0
速度方向相对吊舱光轴向下修正
```

### 4.6 人工接管验证

末制导中切 Position：

```text
QGC 或遥控器切 Position
```

期望：

```text
vehicle_status.nav_state 不再是 14
attack_vision 停止末制导控制
飞机进入 Position 悬停
```

再切 Offboard：

```text
如果 lock_active=True 且 frame_valid=True，attack_vision 恢复末制导
```

失锁或帧超时：

```text
lock_active=False 或 frame_valid=False
attack_vision 不发布有效末制导控制
```

### 4.7 实机测试结束

退出末制导后：

```sh
attack_vision stop
listener vehicle_status
listener trajectory_setpoint
```

确认飞机已回到可控模式，且不再持续收到 attack_vision 的末制导速度设定。

## 5. 常见判断误区

### 5.1 target_vec_ned 不是吊舱坐标

`target_vec_ned_x/y/z` 已经是 NED 坐标下的单位目标视线，不是吊舱 FRD 坐标。

如果：

```text
los_gimbal = [1, 0, 0]
gimbal_yaw = 0
```

则：

```text
target_vec_ned = 当前机体前向在 NED 下的方向
```

所以只有机体 yaw 为 0 时，它才接近 `[1, 0, 0]`。

### 5.2 仿真固定相对 yaw 会导致转圈

`AV_SIM_GMB_YAW=9000` 表示吊舱固定在“相对机体右侧 90 度”。如果长时间保持该值，飞机 yaw 又会去追 `target_vec_ned`，目标方向会随机体继续旋转，可能出现原地转圈。

该测试只用于瞬时验证坐标链路；采样后应恢复：

```sh
param set AV_SIM_GMB_YAW 0
```

### 5.3 PX4_INFO 不一定能在 QGC 中看到

WorkQueue 里的 `PX4_INFO` 不一定稳定显示在 QGC MAVLink Console。验证时优先使用：

```sh
attack_vision status
listener attack_vision_status
listener trajectory_setpoint
```
