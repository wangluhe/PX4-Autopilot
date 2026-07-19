# attack_vision 独立验证流程

本文档用于验证 `src/modules/attack_vision` 在不启动上位机时的独立工作能力，覆盖：

- SITL 仿真测试
- 实机地面静态上电无桨测试
- 实机低速起飞后功能测试

本文默认 TELEM2 对应实机串口 `/dev/ttyS5`，吊舱串口波特率为 `115200`。

## 1. 当前坐标和参数约定

### 1.1 吊舱数据定义

已实测定义：

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

当前算法按以下链路计算目标速度：

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

trajectory_setpoint.velocity[0] ~= target_vec_ned_x * AV_FORWARD_V
trajectory_setpoint.velocity[1] ~= target_vec_ned_y * AV_FORWARD_V
trajectory_setpoint.velocity[2] ~= target_vec_ned_z * AV_FORWARD_V，并受 AV_MAX_VZ 限幅
```

阶段二后的姿态链路为：

```text
q_heading(vehicle_yaw)
* q_mount_yaw(AV_MNT_YAW)
* q_gimbal_yaw_pitch
* los_gimbal
```

即只使用飞机当前 yaw/航向，不再把飞机 roll/pitch 叠加进吊舱光轴方向。

### 1.2 推荐基础参数

独立测试不启动上位机，必须使用 Standalone：

```sh
param set AAATTKVIS_EN 1
param set AV_EXT_MODE 0
param set AV_BAUD 115200
```

按当前实测符号设置：

```sh
param set AV_PIX_X_INV 1
param set AV_PIX_Y_INV 0
param set AV_GMB_YAW_INV 0
param set AV_GMB_PIT_INV 0
param set AV_GMB_ROLL_INV 1
```

当前吊舱机械 yaw 零位与机头航向重合时：

```sh
param set AV_MNT_YAW 0
```

如果以后吊舱机械 yaw 零位相对机头向右安装偏 `90 deg`，则设置：

```sh
param set AV_MNT_YAW 90
```

首次验证建议使用小速度：

```sh
param set AV_FORWARD_V 0.2
param set AV_MAX_VZ 0.2
```

实机飞行确认稳定后，再逐步提高，例如：

```sh
param set AV_FORWARD_V 0.5
param set AV_MAX_VZ 0.3
```

参数确认：

```sh
param show AAATTKVIS_EN
param show AV_EXT_MODE
param show AV_BAUD
param show AV_PIX_X_INV
param show AV_PIX_Y_INV
param show AV_GMB_YAW_INV
param show AV_GMB_PIT_INV
param show AV_GMB_ROLL_INV
param show AV_MNT_YAW
param show AV_FORWARD_V
param show AV_MAX_VZ
```

如需保存到飞控，下次上电自动加载：

```sh
param save
```

## 2. 通用观测指令

PX4 shell 或 QGC MAVLink Console 中使用：

```sh
attack_vision status
listener attack_vision_status
listener trajectory_setpoint
listener offboard_control_mode
listener vehicle_status
listener vehicle_attitude
```

重点看 `attack_vision_status`：

```text
frame_valid: 串口帧是否新鲜有效
frame_age_ms: 最近有效帧年龄，正常应小于 200
lock_active: 吊舱是否锁定目标
rc_offboard: 当前是否处于 Offboard
allow_takeover: 当前是否允许 attack_vision 接管
module_state: 0=HOLD, 1=SWITCHING_TO_OFFBOARD, 2=OFFBOARD

pix_offset_x/y: 像素脱靶量
gimbal_*_raw_deg100: 吊舱原始角度，单位 deg*100
gimbal_*_rad: 经过 AV_GMB_*_INV 转换后的弧度值
los_gimbal_*: 相机/吊舱视线方向
target_vec_ned_*: 转到 NED 后的单位目标方向
```

注意：未解锁、未进入 Offboard、未允许接管时，`los_gimbal_valid` 和 `target_vec_ned_valid` 可能为 `False`，这是正常的。地面静态测试重点看串口帧、像素、吊舱角和符号。

## 3. SITL 仿真测试

### 3.1 编译

```sh
ninja -C build/px4_sitl_default
```

启动 SITL。按工程现有方式启动即可，只要求启动 PX4，不启动上位机。

### 3.2 设置仿真参数

进入 `pxh>` 后执行：

```sh
param set AAATTKVIS_EN 1
param set AV_EXT_MODE 0
param set AV_BAUD 115200

param set AV_PIX_X_INV 1
param set AV_PIX_Y_INV 0
param set AV_GMB_YAW_INV 0
param set AV_GMB_PIT_INV 0
param set AV_GMB_ROLL_INV 1
param set AV_MNT_YAW 0

param set AV_FORWARD_V 0.2
param set AV_MAX_VZ 0.2

param set AV_SIM_PIX_EN 1
param set AV_SIM_GMB_EN 1
param set AV_SIM_PIX_X 0
param set AV_SIM_PIX_Y 0
param set AV_SIM_GMB_ROLL 0
param set AV_SIM_GMB_PIT 0
param set AV_SIM_GMB_YAW 0
```

确认参数：

```sh
param show AAATTKVIS_EN
param show AV_EXT_MODE
param show AV_FORWARD_V
param show AV_MAX_VZ
param show AV_MNT_YAW
param show AV_SIM_PIX_EN
param show AV_SIM_GMB_EN
```

启动模块：

```sh
attack_vision stop
attack_vision start
attack_vision status
listener attack_vision_status
```

期望：

```text
frame_valid: True
frame_age_ms < 200
lock_active: True
pix_offset_x: 0
pix_offset_y: 0
gimbal_*_raw_deg100: 0
```

### 3.3 起飞并进入 Offboard 验证状态

```sh
commander arm
commander takeoff
```

等待高度稳定后观察：

```sh
listener vehicle_status
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
vehicle_status.arming_state: 2
attack_vision_status.frame_valid: True
attack_vision_status.lock_active: True
attack_vision_status.allow_takeover: True
attack_vision_status.module_state: 2
vehicle_status.nav_state: 14
```

`vehicle_status.nav_state=14` 表示 Offboard。

### 3.4 像素脱靶量测试矩阵

中心：

```sh
param set AV_SIM_PIX_X 0
param set AV_SIM_PIX_Y 0
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
los_gimbal_x ~= 1
los_gimbal_y ~= 0
los_gimbal_z ~= 0
```

画面右侧：

```sh
param set AV_SIM_PIX_X -200
param set AV_SIM_PIX_Y 0
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
pix_offset_x: -200
los_gimbal_y > 0
```

画面左侧：

```sh
param set AV_SIM_PIX_X 200
param set AV_SIM_PIX_Y 0
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
pix_offset_x: 200
los_gimbal_y < 0
```

画面下方：

```sh
param set AV_SIM_PIX_X 0
param set AV_SIM_PIX_Y 200
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
pix_offset_y: 200
los_gimbal_z > 0
```

画面上方：

```sh
param set AV_SIM_PIX_X 0
param set AV_SIM_PIX_Y -200
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
pix_offset_y: -200
los_gimbal_z < 0
```

恢复中心：

```sh
param set AV_SIM_PIX_X 0
param set AV_SIM_PIX_Y 0
```

### 3.5 吊舱 yaw/pitch 到 NED 速度测试

中心 + 吊舱零位：

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
gimbal_yaw_rad ~= 0
gimbal_pitch_rad ~= 0
los_gimbal ~= [1, 0, 0]
target_vec_ned ~= 当前飞机航向在 NED 水平面的方向
trajectory_setpoint.velocity ~= target_vec_ned * AV_FORWARD_V
```

吊舱向右 yaw +90 deg：

```sh
param set AV_SIM_GMB_YAW 9000
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
gimbal_yaw_rad ~= +1.5708
target_vec_ned ~= 当前G0水平系右向转到 NED 后的方向
trajectory_setpoint.yaw ~= atan2(target_vec_ned_y, target_vec_ned_x)
```

采样后恢复：

```sh
param set AV_SIM_GMB_YAW 0
```

吊舱上抬 pitch +10 deg：

```sh
param set AV_SIM_GMB_PIT 1000
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
gimbal_pitch_rad > 0
target_vec_ned_z < 0
velocity[2] < 0
```

吊舱下俯 pitch -10 deg：

```sh
param set AV_SIM_GMB_PIT -1000
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
gimbal_pitch_rad < 0
target_vec_ned_z > 0
velocity[2] > 0
```

恢复：

```sh
param set AV_SIM_GMB_PIT 0
```

### 3.6 roll 图像修正测试

roll 只修正图像平面的像素方向，中心点绕光轴滚转后不会改变方向，所以必须配合非中心像素测试。

先设置画面下方目标：

```sh
param set AV_SIM_PIX_X 0
param set AV_SIM_PIX_Y 200
param set AV_SIM_GMB_ROLL 0
listener attack_vision_status
```

记录：

```text
los_gimbal_y
los_gimbal_z
target_vec_ned_y
target_vec_ned_z
```

再注入光轴右滚 raw -10 deg：

```sh
param set AV_SIM_GMB_ROLL -1000
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
gimbal_roll_raw_deg100: -1000
gimbal_roll_rad > 0
los_gimbal_y/z 相比 roll=0 时发生旋转耦合变化
```

恢复：

```sh
param set AV_SIM_GMB_ROLL 0
param set AV_SIM_PIX_X 0
param set AV_SIM_PIX_Y 0
```

### 3.7 安装 yaw 参数测试

模拟吊舱机械 yaw 零位相对机头向右偏 90 deg：

```sh
param set AV_MNT_YAW 90
param set AV_SIM_GMB_YAW 0
param set AV_SIM_GMB_PIT 0
param set AV_SIM_PIX_X 0
param set AV_SIM_PIX_Y 0
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
target_vec_ned 指向当前飞机航向右侧 90 deg
trajectory_setpoint.yaw 指向该 target_vec_ned 方向
```

恢复当前实机安装默认：

```sh
param set AV_MNT_YAW 0
```

### 3.8 Offboard 接管和退出测试

在 Offboard 末制导中切 Position：

```text
QGC 或遥控器切 Position
```

观察：

```sh
listener vehicle_status
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
vehicle_status.nav_state 不再是 14
attack_vision 停止末制导控制
飞机进入位置控制/悬停
```

再切回 Offboard：

```text
QGC 或遥控器切 Offboard
```

期望：

```text
vehicle_status.nav_state: 14
attack_vision_status.module_state: 2
trajectory_setpoint 恢复按 target_vec_ned 发布速度
```

### 3.9 结束仿真

```sh
param set AV_SIM_PIX_EN 0
param set AV_SIM_GMB_EN 0
param set AV_SIM_PIX_X 0
param set AV_SIM_PIX_Y 0
param set AV_SIM_GMB_ROLL 0
param set AV_SIM_GMB_PIT 0
param set AV_SIM_GMB_YAW 0
attack_vision stop
```

## 4. 实机地面静态上电无桨测试

### 4.1 安全准备

必须满足：

```text
不安装桨叶
机体固定或放置稳定
电池电量充足
TELEM2 接吊舱串口，波特率 115200
不启动上位机
测试过程中默认不执行 commander arm
```

### 4.2 上电和参数设置

上电后进入 QGC MAVLink Console 或 NSH，执行：

```sh
param set AAATTKVIS_EN 1
param set AV_EXT_MODE 0
param set AV_BAUD 115200

param set AV_PIX_X_INV 1
param set AV_PIX_Y_INV 0
param set AV_GMB_YAW_INV 0
param set AV_GMB_PIT_INV 0
param set AV_GMB_ROLL_INV 1
param set AV_MNT_YAW 0

param set AV_FORWARD_V 0.2
param set AV_MAX_VZ 0.2

param set AV_SIM_PIX_EN 0
param set AV_SIM_GMB_EN 0
```

确认：

```sh
param show AAATTKVIS_EN
param show AV_EXT_MODE
param show AV_BAUD
param show AV_PIX_X_INV
param show AV_PIX_Y_INV
param show AV_GMB_YAW_INV
param show AV_GMB_PIT_INV
param show AV_GMB_ROLL_INV
param show AV_MNT_YAW
param show AV_FORWARD_V
param show AV_MAX_VZ
```

确认无误后可保存：

```sh
param save
```

### 4.3 启动模块

```sh
attack_vision stop
attack_vision start
attack_vision status
listener attack_vision_status
```

期望：

```text
frame_valid: True
frame_age_ms < 200
module_state: 0
rc_offboard: False
allow_takeover: False
```

未解锁时，不应进入末制导：

```sh
listener vehicle_status
listener trajectory_setpoint
```

期望：

```text
vehicle_status.arming_state 不是 2
vehicle_status.nav_state 不是 14
trajectory_setpoint 不应表现为持续有效末制导速度输出
```

### 4.4 串口帧稳定性测试

吊舱保持正常连接，连续多次查看：

```sh
listener attack_vision_status
listener attack_vision_status
listener attack_vision_status
```

期望：

```text
frame_valid 稳定 True
frame_age_ms 通常约 20~50ms，必须小于 200ms
gimbal_*_raw_deg100 连续更新
timestamp 连续更新
```

短暂断开吊舱串口或停止吊舱输出，再查看：

```sh
listener attack_vision_status
attack_vision status
```

期望：

```text
模块不阻塞
MAVLink Console 仍可输入命令
frame_valid 变为 False
frame_age_ms 增大
```

重新接上吊舱后：

```sh
listener attack_vision_status
listener attack_vision_status
```

期望：

```text
frame_valid 恢复 True
frame_age_ms 恢复到小于 200ms
pix_offset_x/y 和 gimbal_*_raw_deg100 正常更新
```

### 4.5 像素脱靶量方向测试

锁定或点击画面不同位置，查看：

```sh
listener attack_vision_status
```

判断：

```text
点击/锁定画面左侧  -> pix_offset_x > 0
点击/锁定画面右侧  -> pix_offset_x < 0
点击/锁定画面上方  -> pix_offset_y < 0
点击/锁定画面下方  -> pix_offset_y > 0
```

如果方向不一致：

```text
左右反了：调整 AV_PIX_X_INV
上下反了：调整 AV_PIX_Y_INV
```

当前实测默认应保持：

```sh
param set AV_PIX_X_INV 1
param set AV_PIX_Y_INV 0
```

### 4.6 吊舱 yaw/pitch/roll 符号测试

每做一个动作后查看：

```sh
listener attack_vision_status
```

测试矩阵：

```text
吊舱机械归零
  -> gimbal_yaw_raw_deg100 接近 0
  -> gimbal_pitch_raw_deg100 接近 0
  -> gimbal_roll_raw_deg100 接近 0

吊舱向机体右侧转
  -> gimbal_yaw_raw_deg100 增大为正
  -> gimbal_yaw_rad 增大为正

吊舱向机体左侧转
  -> gimbal_yaw_raw_deg100 减小为负
  -> gimbal_yaw_rad 减小为负

吊舱向上抬
  -> gimbal_pitch_raw_deg100 增大为正
  -> gimbal_pitch_rad 增大为正

吊舱向下俯
  -> gimbal_pitch_raw_deg100 减小为负
  -> gimbal_pitch_rad 减小为负

光轴右滚
  -> gimbal_roll_raw_deg100 为负
  -> gimbal_roll_rad 为正

光轴左滚
  -> gimbal_roll_raw_deg100 为正
  -> gimbal_roll_rad 为负
```

若 yaw/pitch/roll 转换后符号不一致，分别调整：

```sh
param set AV_GMB_YAW_INV 0 或 1
param set AV_GMB_PIT_INV 0 或 1
param set AV_GMB_ROLL_INV 0 或 1
```

当前实测默认应保持：

```sh
param set AV_GMB_YAW_INV 0
param set AV_GMB_PIT_INV 0
param set AV_GMB_ROLL_INV 1
```

### 4.7 G0 水平系静态复查

吊舱机械三轴归零，机体水平放置：

```sh
listener attack_vision_status
listener vehicle_attitude
```

期望：

```text
gimbal_yaw_raw_deg100 接近 0
gimbal_pitch_raw_deg100 接近 0
gimbal_roll_raw_deg100 接近 0
vehicle_attitude Roll/Pitch 接近 0
```

保持吊舱机械归零，仅把机头向上抬：

```sh
listener attack_vision_status
listener vehicle_attitude
```

期望：

```text
vehicle_attitude Pitch 增大
gimbal_pitch_raw_deg100 也随之增大
```

保持吊舱机械归零，仅水平旋转机体 yaw：

```sh
listener attack_vision_status
listener vehicle_attitude
```

期望：

```text
vehicle_attitude Yaw 变化
gimbal_yaw_raw_deg100 仍接近 0
```

这说明当前吊舱 yaw/pitch 是相对 G0 水平系的输出，算法不应再乘完整机体 roll/pitch。

### 4.8 停止模块

```sh
attack_vision stop
attack_vision status
```

期望模块进入停止或暂停末制导状态，地面静态测试结束。

## 5. 实机起飞后低速测试

### 5.1 起飞前确认

正式飞行前重新确认：

```sh
param show AAATTKVIS_EN
param show AV_EXT_MODE
param show AV_FORWARD_V
param show AV_MAX_VZ
param show AV_MNT_YAW
attack_vision status
listener attack_vision_status
```

推荐首次飞行速度：

```sh
param set AV_FORWARD_V 0.2
param set AV_MAX_VZ 0.2
```

确认：

```text
frame_valid: True
frame_age_ms < 200
lock_active 可随目标锁定正常变化
未解锁时 allow_takeover: False
```

### 5.2 起飞和悬停

可以使用 QGC 正常起飞并切到 Position，也可以通过 PX4 shell：

```sh
commander arm
commander takeoff
```

等待飞机稳定悬停后查看：

```sh
listener vehicle_status
listener vehicle_attitude
listener attack_vision_status
```

期望：

```text
vehicle_status.arming_state: 2
frame_valid: True
frame_age_ms < 200
```

### 5.3 锁定目标但暂不进入末制导

在 Position 中让吊舱锁定目标，观察：

```sh
listener attack_vision_status
```

期望：

```text
lock_active: True
pix_offset_x/y 正常更新
gimbal_yaw_raw_deg100 / gimbal_pitch_raw_deg100 正常更新
```

此时如果还没有进入 Offboard：

```text
rc_offboard: False
module_state: 0
target_vec_ned_valid 可能为 False
```

这是正常的。

### 5.4 进入末制导

切入 Offboard 或触发你的 RC Offboard 档位后，观察：

```sh
listener vehicle_status
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
vehicle_status.nav_state: 14
attack_vision_status.rc_offboard: True
attack_vision_status.allow_takeover: True
attack_vision_status.module_state: 2
attack_vision_status.lock_active: True
attack_vision_status.frame_valid: True
attack_vision_status.los_gimbal_valid: True
attack_vision_status.target_vec_ned_valid: True
```

速度判断：

```text
trajectory_setpoint.velocity[0] ~= target_vec_ned_x * AV_FORWARD_V
trajectory_setpoint.velocity[1] ~= target_vec_ned_y * AV_FORWARD_V
trajectory_setpoint.velocity[2] ~= target_vec_ned_z * AV_FORWARD_V，并受 AV_MAX_VZ 限幅
```

### 5.5 人工退出和恢复

末制导过程中切 Position：

```text
QGC 或遥控器切 Position
```

观察：

```sh
listener vehicle_status
listener attack_vision_status
listener trajectory_setpoint
```

期望：

```text
vehicle_status.nav_state 不再是 14
attack_vision 停止末制导
飞机进入 Position 悬停
```

再切回 Offboard：

```text
QGC 或遥控器切 Offboard
```

期望：

```text
若 lock_active=True 且 frame_valid=True
attack_vision 恢复末制导
trajectory_setpoint 恢复按 target_vec_ned 发布速度
```

### 5.6 失锁和帧超时保护

飞行中短暂取消目标锁定：

```sh
listener attack_vision_status
```

期望：

```text
lock_active: False
target_vec_ned_valid: False
停止有效末制导控制
```

吊舱串口异常或无新帧：

```text
frame_valid: False
frame_age_ms 增大
停止有效末制导控制
```

### 5.7 结束实机飞行测试

退出末制导并回到 Position：

```text
QGC 或遥控器切 Position
```

停止模块：

```sh
attack_vision stop
attack_vision status
```

降落：

```sh
commander land
```

## 6. 常见判断误区

### 6.1 `target_vec_ned ~= [1, 0, 0]` 的条件

只有当以下条件同时满足时，`target_vec_ned` 才接近 `[1, 0, 0]`：

```text
飞机 yaw 接近 0
AV_MNT_YAW = 0
gimbal_yaw = 0
gimbal_pitch = 0
pix_offset_x = 0
pix_offset_y = 0
```

如果飞机 yaw 不是 0，中心目标和吊舱零位时：

```text
target_vec_ned ~= 当前飞机航向在 NED 水平面的方向
```

### 6.2 仿真固定 yaw 可能导致原地转圈

`AV_SIM_GMB_YAW=9000` 表示吊舱光轴固定在 G0 水平系右侧 `90 deg`。如果长时间保持该值，同时控制又命令飞机 yaw 对准 `target_vec_ned`，目标方向会随机体继续旋转，可能出现原地转圈。

该项只用于瞬时采样验证，采样后恢复：

```sh
param set AV_SIM_GMB_YAW 0
```

### 6.3 QGC 看不到 `PX4_INFO`

WorkQueue 内的 `PX4_INFO` 不一定稳定显示在 QGC MAVLink Console。验证时优先使用：

```sh
attack_vision status
listener attack_vision_status
listener trajectory_setpoint
```
