# 图像末制导模块（attack_vision）使用说明

## 一、功能概述

该模块实现基于吊舱图像反馈的末制导功能：

1. **串口通信**：通过TELEM2口（/dev/ttyS2）接收吊舱64字节反馈帧（每40ms一次）
2. **数据解析**：解析锁定状态（第5-6字节Bit9~Bit10）和目标脱靶量（第59-62字节）
3. **自动制导**：当锁定有效时，自动切换到Offboard模式并发布速度控制指令
4. **安全保护**：失锁或超时（200ms无数据）时，自动切换回悬停模式

## 二、硬件连接

### 2.1 串口连接
- **飞控端口**：TELEM2（对应6xrt板子的/dev/ttyS2）
- **吊舱串口**：连接到飞控TELEM2口
- **波特率**：默认115200（可通过参数AV_BAUD修改）
- **电平**：3.3V或5V（根据吊舱规格）

### 2.2 连接示意图
```
吊舱串口 TX  -----> 飞控 TELEM2 RX
吊舱串口 RX  <----- 飞控 TELEM2 TX
吊舱串口 GND -----> 飞控 GND
```

## 三、编译配置

### 3.1 确保模块已启用

模块已配置在 `boards/px4/fmu-v6xrt/default.px4board` 文件中：
```
CONFIG_MODULES_ATTACK_VISION=y
```

### 3.2 编译固件

在PX4源码根目录执行：
```bash
# 清理之前的编译文件（可选）
make px4_fmu-v6xrt_default clean

# 编译固件
make px4_fmu-v6xrt_default
```

编译完成后，固件文件位于：
```
build/px4_fmu-v6xrt_default/px4_fmu-v6xrt_default.px4
```

### 3.3 验证模块编译

检查编译输出中是否包含：
```
-- Building modules/attack_vision
```

或在编译完成后检查：
```bash
# 检查模块是否在固件中
strings build/px4_fmu-v6xrt_default/px4_fmu-v6xrt_default.elf | grep attack_vision
```

## 四、烧录固件

### 4.1 使用QGroundControl烧录

1. 打开QGroundControl
2. 连接飞控（USB或串口）
3. 进入 **Setup** -> **Firmware**
4. 选择 **Custom firmware file**
5. 选择编译生成的 `.px4` 文件：
   ```
   build/px4_fmu-v6xrt_default/px4_fmu-v6xrt_default.px4
   ```
6. 点击 **Flash** 开始烧录

### 4.2 使用命令行烧录

```bash
# 方法1：使用px_uploader.py（需要安装PX4工具链）
python Tools/uploader.py --port /dev/ttyACM0 build/px4_fmu-v6xrt_default/px4_fmu-v6xrt_default.px4

# 方法2：使用make upload（需要配置环境）
make px4_fmu-v6xrt_default upload
```

**注意**：替换 `/dev/ttyACM0` 为实际的串口设备名（Linux）或COM口（Windows）。

## 五、参数配置

### 5.1 必要参数

通过QGroundControl或MAVLink命令行设置以下参数：

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `AAATTKVIS_EN` | INT32 | 1 | 模块使能开关（0=禁用，1=启用） |
| `AV_BAUD` | INT32 | 115200 | 串口波特率（9600/19200/38400/57600/115200） |
| `AV_KP` | FLOAT | 0.001 | 速度控制增益（m/s每像素） |
| `AV_DEAD` | FLOAT | 5.0 | 像素死区阈值（像素，小于此值不产生控制） |
| `AV_MAX_V` | FLOAT | 1.0 | 最大速度限制（m/s） |

### 5.2 参数设置方法

#### 方法1：QGroundControl
1. 连接飞控
2. 进入 **Parameters** 界面
3. 搜索参数名（如 `AAATTKVIS_EN`）
4. 修改参数值
5. 点击 **Save** 保存

#### 方法2：MAVLink命令行
```bash
# 连接飞控（通过USB或串口）
mavlink_shell.py /dev/ttyACM0

# 在shell中执行
param set AAATTKVIS_EN 1
param set AV_BAUD 115200
param set AV_KP 0.001
param set AV_DEAD 5.0
param set AV_MAX_V 1.0

# 保存参数
param save
```

### 5.3 参数调优建议

- **AV_KP**（控制增益）：
  - 值过大：响应过快，可能震荡
  - 值过小：响应过慢，跟踪滞后
  - 建议范围：0.0005 ~ 0.002（根据飞行速度和目标距离调整）

- **AV_DEAD**（死区）：
  - 值过大：小偏差不响应，跟踪精度低
  - 值过小：噪声敏感，可能抖动
  - 建议范围：3 ~ 10像素

- **AV_MAX_V**（最大速度）：
  - 根据飞行安全要求设置
  - 建议范围：0.5 ~ 2.0 m/s

## 六、模块启动

### 6.1 自动启动

模块已配置在 `ROMFS/px4fmu_common/init.d/rc.mc_apps` 中自动启动：
```bash
if param greater -s AAATTKVIS_EN 0
then
    attack_vision start
fi
```

**说明**：当 `AAATTKVIS_EN > 0` 时，飞控上电后自动启动模块。

### 6.2 手动启动/停止

通过MAVLink命令行：
```bash
# 启动模块
attack_vision start

# 停止模块
attack_vision stop

# 查看模块状态
attack_vision status
```

## 七、试验操作流程

### 7.1 试验前准备

1. **硬件检查**：
   - [ ] 吊舱串口正确连接到飞控TELEM2
   - [ ] 串口电平匹配（3.3V/5V）
   - [ ] 串口GND已连接

2. **参数配置**：
   - [ ] 设置 `AAATTKVIS_EN = 1`（启用模块）
   - [ ] 设置 `AV_BAUD`（与吊舱波特率一致）
   - [ ] 调整 `AV_KP`、`AV_DEAD`、`AV_MAX_V`（根据实际情况）

3. **飞控配置**：
   - [ ] 允许Offboard模式（参数 `COM_OF_LOSS_T` > 0）
   - [ ] 设置Offboard超时时间（建议5秒）
   - [ ] 检查安全开关和紧急停止功能

4. **吊舱检查**：
   - [ ] 吊舱上电正常
   - [ ] 吊舱能正常发送64字节反馈帧
   - [ ] 帧格式符合协议规范

### 7.2 地面测试

1. **启动模块**：
   ```bash
   attack_vision start
   ```

2. **检查模块状态**：
   ```bash
   attack_vision status
   ```
   应该看到类似输出：
   ```
   attack_vision running, fd=3, lock=0, pix=(0,0)
   ```
   - `fd=3`：串口文件描述符（>0表示打开成功）
   - `lock=0`：锁定状态（0=未锁定，1=已锁定）
   - `pix=(0,0)`：像素脱靶量（方位，俯仰）

3. **检查串口数据**：
   ```bash
   # 查看模块日志
   listener attack_vision
   ```
   或通过QGroundControl查看日志。

4. **验证帧接收**：
   - 观察 `pix` 值是否有变化（表示接收到帧）
   - 检查是否有帧校验错误（日志中会显示）

### 7.3 飞行测试

**重要安全提示**：
- 首次测试建议在安全空旷场地进行
- 保持安全距离（建议>10米）
- 准备紧急停止方案（遥控器或地面站）
- 建议先进行悬停测试，确认模块工作正常

#### 测试步骤：

1. **起飞前检查**：
   - [ ] 飞控已解锁
   - [ ] GPS定位正常
   - [ ] 模块已启动（`attack_vision status`）
   - [ ] 吊舱串口连接正常（`fd > 0`）

2. **起飞到安全高度**：
   - 手动或自动起飞到5-10米高度
   - 切换到悬停模式（Loiter）

3. **激活跟踪模式**：
   - 在遥控器上操作吊舱进入跟踪模式
   - 吊舱识别并锁定目标后，会发送锁定状态（Bit9~Bit10 = 01或10）
   - 模块检测到锁定后，自动切换到Offboard模式

4. **观察制导效果**：
   - 无人机应自动向目标方向移动
   - 通过QGroundControl观察速度指令和位置变化
   - 观察像素脱靶量是否逐渐减小

5. **失锁测试**：
   - 手动退出跟踪模式或遮挡目标
   - 模块应自动切换回悬停模式
   - 无人机应保持当前位置悬停

### 7.4 故障排查

#### 问题1：模块无法启动
- **检查**：`AAATTKVIS_EN` 参数是否为1
- **检查**：串口设备是否正确（默认/dev/ttyS2）
- **检查**：是否有其他程序占用串口

#### 问题2：无法接收数据
- **检查**：串口连接是否正确（TX/RX是否接反）
- **检查**：波特率是否匹配（`AV_BAUD`与吊舱一致）
- **检查**：串口电平是否匹配（3.3V/5V）
- **检查**：GND是否连接

#### 问题3：帧校验失败
- **检查**：帧格式是否符合协议（64字节，帧头0xFC 0x2C，帧尾0xF0）
- **检查**：串口是否有干扰或数据丢失
- **检查**：波特率是否正确

#### 问题4：锁定后不切换模式
- **检查**：飞控是否允许Offboard模式（`COM_OF_LOSS_T > 0`）
- **检查**：飞控是否已解锁
- **检查**：锁定状态解析是否正确（查看日志）
- **检查**：伺服状态是否为跟踪模式（0x07）

#### 问题5：制导效果不佳
- **调整**：增加 `AV_KP`（响应更快）或减小（更平滑）
- **调整**：减小 `AV_DEAD`（精度更高）或增大（更稳定）
- **检查**：目标距离和速度是否合理
- **检查**：像素脱靶量范围是否正常

## 八、调试工具

### 8.1 查看模块状态
```bash
attack_vision status
```

### 8.2 查看日志
```bash
# 方法1：通过MAVLink
listener attack_vision

# 方法2：通过QGroundControl
# 进入 Logs 界面查看

# 方法3：通过串口终端
# 连接飞控串口，查看实时输出
```

### 8.3 监控uORB话题
```bash
# 查看Offboard控制模式
listener offboard_control_mode

# 查看轨迹设定点（速度指令）
listener trajectory_setpoint

# 查看载具状态
listener vehicle_status

# 查看载具命令
listener vehicle_command
```

### 8.4 参数监控
```bash
# 查看所有AV_开头的参数
param show AV_*

# 实时监控参数变化
param monitor AAATTKVIS_EN
param monitor AV_KP
```

## 九、安全注意事项

1. **模式切换**：
   - 模块会自动切换模式，但建议在测试时保持手动控制能力
   - 确保遥控器紧急停止功能可用

2. **速度限制**：
   - `AV_MAX_V` 参数限制最大速度，建议根据实际情况设置
   - 首次测试建议设置较小值（如0.5 m/s）

3. **超时保护**：
   - 200ms无数据会自动切换回悬停
   - 但仍需确保串口连接可靠

4. **Offboard模式安全**：
   - 确保 `COM_OF_LOSS_T` 参数设置合理（建议5秒）
   - Offboard模式丢失后会触发安全模式

5. **测试环境**：
   - 首次测试建议在空旷场地
   - 保持安全距离
   - 准备紧急停止方案

## 十、技术细节

### 10.1 帧格式说明

64字节反馈帧格式（参考attack_vision.md）：
- 字节0-1：帧头（0xFC 0x2C）
- 字节5-6：吊舱状态（Bit9~Bit10为锁定标识）
- 字节9：伺服状态（0x07=跟踪模式）
- 字节59-60：目标脱靶量-方位方向（INT16，像素）
- 字节61-62：目标脱靶量-俯仰方向（INT16，像素）
- 字节62：异或校验（第3~62字节异或）
- 字节63：帧尾（0xF0）

### 10.2 控制逻辑

1. **像素偏差 → 速度指令**：
   ```
   vx = -AV_KP * pixel_offset_x  (前向速度)
   vy = -AV_KP * pixel_offset_y  (横向速度)
   ```

2. **死区处理**：
   ```
   if |pixel_offset| < AV_DEAD:
       pixel_offset = 0
   ```

3. **速度限幅**：
   ```
   v = constrain(v, -AV_MAX_V, AV_MAX_V)
   ```

### 10.3 模式切换逻辑

- **锁定有效 + 数据新鲜** → 切换到Offboard模式 → 发布速度指令
- **失锁或超时** → 切换到Loiter模式 → 悬停

## 十一、常见问题（FAQ）

**Q1: 模块启动后立即退出？**
A: 检查 `AAATTKVIS_EN` 参数是否为1，检查串口是否打开成功。

**Q2: 如何修改串口设备？**
A: 需要修改源码 `attack_vision.cpp` 中的 `open_uart()` 函数，将 `/dev/ttyS2` 改为其他设备。

**Q3: 如何禁用自动启动？**
A: 设置参数 `AAATTKVIS_EN = 0`，或修改启动脚本。

**Q4: 制导时无人机不动？**
A: 检查是否已切换到Offboard模式，检查速度指令是否发布，检查参数 `AV_KP` 是否过小。

**Q5: 如何调整控制方向？**
A: 修改 `handle_guidance()` 函数中的速度计算符号，或调整参数 `AV_KP` 的符号。

---

**版本**：v1.0
**最后更新**：2025年
**作者**：PX4开发团队


## 十二、静态测试（SITL）

本节指导在不接真实吊舱、不起飞的前提下，通过 PX4 SITL、虚拟串口与数据回放验证模块功能。

### 12.1 目标

- 验证帧解析（帧头/尾、锁定位、像素偏差、异或校验）
- 验证控制逻辑（死区、KP、速度限幅）
- 验证模式切换（锁定生效→Offboard，失锁/超时→Loiter）
- 验证 uORB 输出（`offboard_control_mode`、`trajectory_setpoint` 等）

### 12.2 前置条件

- 已安装 PX4 工具链（1.15）
- Linux 主机可用 `socat`、`python3`
- 已编译 SITL：`make px4_sitl_default` 可用

### 12.3 SITL 下的串口设备路径

源码已在 SITL/Posix 下将串口设备固定为 `/tmp/attack_vision_tty`，硬件保持 `/dev/ttyS5`。对应实现参见 `open_uart()`：

- SITL/Posix：`/tmp/attack_vision_tty`
- 硬件：`/dev/ttyS5`

### 12.4 创建虚拟串口（PTY）

**方法1：使用 Python 脚本（推荐）**

在终端运行并保持：

```bash
cd /home/www/px4_wlh/px4-6xmain-1.15/src/modules/attack_vision
python3 create_virtual_uart.py
```

这个脚本会：
- 创建虚拟串口对（主从 PTY）
- 创建符号链接 `/tmp/attack_vision_tty`（模块端）
- 创建命名管道（FIFO）`/tmp/attack_vision_fifo`（测试脚本端）
- 实时转发数据并显示传输状态

**方法2：使用 socat（备选，可能有兼容性问题）**

如果 Python 方法不可用，可以尝试 socat：

```bash
socat -d -d pty,raw,echo=0,link=/tmp/attack_vision_tty \
             pty,raw,echo=0,link=/tmp/attack_vision_src
```

**注意**：socat 在某些系统上可能无法正常工作，推荐使用方法1。

- `/tmp/attack_vision_tty`：供模块在 SITL 中打开（符号链接到 slave PTY）
- `/tmp/attack_vision_fifo`：供测试脚本写入（命名管道，数据会转发到 master PTY）

### 12.5 启动 SITL 并配置参数

```bash
make px4_sitl_default none
```

进入 PX4 控制台后：

```bash
param set AAATTKVIS_EN 1
param set AV_BAUD 115200
param set AV_KP 0.001
param set AV_DEAD 5.0
param set AV_MAX_V 1.0
param save

attack_vision start
attack_vision status
```

期望看到 `fd>0`（串口打开成功）。

### 12.6 发送测试帧（Python 回放）

新建文件 `av_frame_sender.py` 并执行 `python3 av_frame_sender.py`：

```python
#!/usr/bin/env python3
import os
import time
import struct

DEV = "/tmp/attack_vision_src"
HZ = 25.0

def build_frame(locked: bool, pix_x: int, pix_y: int) -> bytes:
    data = bytearray(64)
    data[0] = 0xFC
    data[1] = 0x2C

    status = 0
    if locked:
        status |= (1 << 9)  # Bit9=1 表示 01 锁定示例
    data[4:6] = struct.pack('<H', status)  # 第5-6字节，小端

    data[8] = 0x07 if locked else 0x00     # 第9字节：伺服状态

    data[58:60] = struct.pack('<h', int(pix_x))  # X: 第59-60字节
    data[60:62] = struct.pack('<h', int(pix_y))  # Y: 第61-62字节

    xor_val = 0
    for b in data[2:62]:
        xor_val ^= b
    data[62] = xor_val & 0xFF              # 校验：第63字节

    data[63] = 0xF0                        # 帧尾：第64字节
    return bytes(data)

def main():
    fd = os.open(DEV, os.O_RDWR | os.O_SYNC)
    try:
        t = 0.0
        dt = 1.0 / HZ
        while True:
            pix_x = 50 if (int(t) % 4) < 2 else -50
            pix_y = 30 if (int(t / 2) % 4) < 2 else -30
            locked = (int(t) % 10) < 5      # 每5秒锁定/未锁定切换
            frame = build_frame(locked, pix_x, pix_y)
            os.write(fd, frame)
            time.sleep(dt)
            t += dt
    finally:
        os.close(fd)

if __name__ == "__main__":
    main()
```

### 12.7 观测与验证

在 SITL 控制台：

```bash
attack_vision status
listener vehicle_status
listener offboard_control_mode
listener trajectory_setpoint
```

期望：
- 未锁定 → 不进入 Offboard 或回到 Loiter；速度设定点为 0/NaN
- 锁定且偏差超出 `AV_DEAD` → 进入 Offboard；速度随像素变化且限幅 `AV_MAX_V`
- 停止回放 ≥200ms 或清除锁定 → 返回 Loiter

### 12.8 排错要点

- `attack_vision` 命令不存在：确认模块已被 SITL 编译（一般默认包含），重新构建 `make px4_sitl_default none`
- 串口打开失败：确认 `socat` 正在运行并生成 `/tmp/attack_vision_tty`
- 无速度输出：检查 `AV_*` 参数、锁定位（Bit9~10）、伺服状态（0x07）
- 校验失败：确认异或范围索引 `[2..61]`，校验位索引 `62`，帧尾 `0xF0`

