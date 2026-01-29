#include "vserial.h"
#include <px4_platform_common/log.h>
#include <cmath>
#include <lib/mathlib/mathlib.h>

VSerial& VSerial::get_instance()
{
	static VSerial instance;
	return instance;
}

VSerial::~VSerial()
{
    // 清理资源
}

int VSerial::init()
{
	if (_initialized) {
		return 0;
	}

	// 使用当前时间作为随机种子，确保每次运行都不同
	uint64_t seed = hrt_absolute_time();
	_random_engine.seed(static_cast<unsigned int>(seed));

	// 生成随机姿态角度
	generate_random_attitude();

    	// 在SITL模式下，我们模拟一个虚拟串口
	_initialized = true;
	_last_sim_time = hrt_absolute_time();
	_last_control_time = _last_sim_time;

	PX4_INFO("Virtual serial port initialized for SITL simulation");

	PX4_INFO("随机吊舱姿态: 横滚=%.2f°, 俯仰=%.2f°, 方位=%.2f°",
        static_cast<double>(_random_roll_deg100) / 100.0,
        static_cast<double>(_random_pitch_deg100) / 100.0,
        static_cast<double>(_random_yaw_deg100) / 100.0);

	return 0;
}

void VSerial::generate_random_attitude()
{
	_random_roll_deg100 = _roll_distribution(_random_engine);
	_random_pitch_deg100 = _pitch_distribution(_random_engine);
	_random_yaw_deg100 = _yaw_distribution(_random_engine);
}

// 新增：获取当前随机生成的吊舱姿态
void VSerial::get_gimbal_attitude(float &roll_deg, float &pitch_deg, float &yaw_deg) const
{
	roll_deg = _random_roll_deg100 / 100.0f;
	pitch_deg = _random_pitch_deg100 / 100.0f;
	yaw_deg = _random_yaw_deg100 / 100.0f;
}


// 新增：设置控制量输入
// 在vserial.cpp中修改set_control_input
void VSerial::set_control_input(float vx, float vy, float vz)
{
	_last_vx = vx;
	_last_vy = vy;
	_last_vz = vz;
	_last_control_time = hrt_absolute_time();

	PX4_DEBUG("VSerial received control: vx=%.3f, vy=%.3f, vz=%.3f",
			(double)vx, (double)vy, (double)vz);
}

// 新增：获取当前脱靶量
void VSerial::get_target_offset(int16_t &x, int16_t &y) const
{
	x = _current_offset_x;
	y = _current_offset_y;
}

int VSerial::write(const uint8_t *data, size_t length)
{
	if (!_initialized) {
		return -1;
	}

	PX4_DEBUG("VSerial write: %zu bytes", length);
	// 在仿真中，写入数据可以用于调试，但通常不需要实际处理
	return length;
}

int VSerial::read(uint8_t *buffer, size_t length)
{
	if (!_initialized) {
		return -1;
	}

	// 生成模拟数据（包含闭环控制）
	generate_sim_data();

	if (_sim_data_available > 0) {
		size_t copy_size = _sim_data_available;
		if (copy_size > length) {
			copy_size = length;
		}

		memcpy(buffer, _sim_buffer, copy_size);
		_sim_data_available = 0;

		PX4_DEBUG("VSerial read: %zu bytes, offset=(%d,%d)",
                copy_size, _current_offset_x, _current_offset_y);
		return copy_size;
	}

	return 0;
}

void VSerial::generate_sim_data()
{
	hrt_abstime now = hrt_absolute_time();

	// 每40ms生成一次数据（25Hz）
	if (now - _last_sim_time < 40000) {
		return;
	}

	// float dt = (now - _last_sim_time) / 1e6f; // 转换为秒
	_last_sim_time = now;

	// ========== 闭环控制逻辑 ==========
	// 根据新的控制策略更新脱靶量：
	// - 前向速度 vx 会减小目标距离（这里用脱靶量模拟）
	// - 横向速度 vy 会影响方向偏差
	// - 垂直速度 vz 会影响俯仰偏差

	// 1. 自然衰减（目标有向中心移动的趋势）
	_current_offset_x *= DECAY_RATE;
	_current_offset_y *= DECAY_RATE;

	// 2. 控制响应：根据新的控制策略
	// 横向控制影响方向偏差
	_current_offset_x -= _last_vy * CONTROL_GAIN * 10.0f;
	// 垂直控制影响俯仰偏差
	_current_offset_y -= _last_vz * CONTROL_GAIN * 10.0f;
	// 前向控制会同时减小两个方向的偏差（模拟靠近目标）
	_current_offset_x *= (1.0f - fabsf(_last_vx) * 0.01f);
	_current_offset_y *= (1.0f - fabsf(_last_vx) * 0.01f);

	// 3. 添加一些随机扰动，模拟目标移动和环境噪声
	float noise_x = (rand() % 21 - 10) * 0.1f;
	float noise_y = (rand() % 21 - 10) * 0.1f;
	_current_offset_x += noise_x;
	_current_offset_y += noise_y;

	// 4. 限幅保护
	_current_offset_x = (_current_offset_x > MAX_OFFSET) ? MAX_OFFSET :
				((_current_offset_x < -MAX_OFFSET) ? -MAX_OFFSET : _current_offset_x);
	_current_offset_y = (_current_offset_y > MAX_OFFSET) ? MAX_OFFSET :
				((_current_offset_y < -MAX_OFFSET) ? -MAX_OFFSET : _current_offset_y);

	// 5. 如果脱靶量很小，认为已锁定中心
	// bool near_center = (fabsf(_current_offset_x) < 5.0f && fabsf(_current_offset_y) < 5.0f);

	// 模拟视觉吊舱数据包结构
	struct SimPodData {
		uint8_t header1;        // 0xFC
		uint8_t header2;        // 0x2C
		uint8_t payload_type;   // 载荷类型
		uint8_t self_test;      // 自检结果
		uint16_t status1;       // 吊舱状态1
		uint16_t status2;       // 吊舱状态2
		uint8_t servo_status;   // 伺服状态
		int16_t azimuth;        // 方位角 ×100
		int16_t elevation;      // 俯仰角 ×100
		int16_t roll;           // 横滚角 ×100
		uint8_t reserved1[3];   // 备用
		uint8_t target_info;    // 目标编号及类型
		uint8_t reserved2[2];   // 备用
		uint8_t tf_usage;       // TF使用容量百分比
		uint8_t tf_total;       // TF卡总容量
		uint16_t ir_focus;      // 红外焦距 ×10
		uint16_t visible_focus; // 可见光焦距 ×10
		uint16_t target_id;     // 目标识别编号
		float target_lon;       // 目标经度
		float target_lat;       // 目标纬度
		int16_t target_alt;     // 目标海拔高度
		int16_t soc_temp;       // SOC处理器温度
		int16_t gyro_azimuth;   // 陀螺方位角速度 ×100
		int16_t gyro_elevation; // 陀螺俯仰角速度 ×100
		int16_t gyro_roll;      // 陀螺横滚角速度 ×100
		uint8_t display_mode;   // 当前显示图像反馈
		uint8_t reserved3[2];   // 备用
		int16_t track_point_x;  // 目标跟踪点宽方向坐标
		int16_t track_point_y;  // 目标跟踪点高方向坐标
		int16_t track_box_w;    // 目标跟踪框宽方向长度
		int16_t track_box_h;    // 目标跟踪框高方向长度
		int16_t off_target_azimuth;  // 目标脱靶量方位方向
		int16_t off_target_elevation;// 目标脱靶量俯仰方向
		uint8_t checksum;       // 异或校验
		uint8_t footer;         // 0xF0
	} __attribute__((packed));

	static SimPodData sim_data{};
	static uint32_t frame_counter = 0;

	frame_counter++;

	// 填充数据包
	sim_data.header1 = 0xFC;
	sim_data.header2 = 0x2C;
	sim_data.payload_type = 0x01;
	sim_data.self_test = 0xFF;  // 自检正常

	// 设置状态：如果接近中心且控制有效，认为锁定
	// bool target_locked = near_center || (fabsf(_last_vx) > 0.01f || fabsf(_last_vy) > 0.01f);
	bool target_locked = true;
	if (target_locked) {
		sim_data.status1 = (1 << 9);  // 设置锁定状态位
		sim_data.servo_status = 0x07; // 跟踪模式
	} else {
		sim_data.status1 = 0;
		sim_data.servo_status = 0x00; // 非跟踪模式
	}

	sim_data.status2 = 0;

	// // 模拟角度数据
	// sim_data.azimuth = 1000;    // 10.00度
	// sim_data.elevation = -500;  // -5.00度
	// sim_data.roll = 0;

	// 使用随机生成的姿态角度数据
	sim_data.azimuth = _random_yaw_deg100;    // 方位角（0~360度）
	sim_data.elevation = _random_pitch_deg100;  // 俯仰角（-30~30度）
	sim_data.roll = _random_roll_deg100;      // 横滚角（-30~30度）


	// 模拟目标信息
	sim_data.target_info = target_locked ? 0x01 : 0x00;

	// 设置脱靶量 - 使用闭环控制计算的值
	sim_data.off_target_azimuth = static_cast<int16_t>(_current_offset_x);
	sim_data.off_target_elevation = static_cast<int16_t>(_current_offset_y);

	// 计算校验和
	sim_data.checksum = 0;
	const uint8_t *data_ptr = reinterpret_cast<const uint8_t*>(&sim_data);
	for (int i = 2; i < 62; i++) {
		sim_data.checksum ^= data_ptr[i];
	}

	sim_data.footer = 0xF0;

	// 复制到输出缓冲区
	memcpy(_sim_buffer, &sim_data, sizeof(sim_data));
	_sim_data_available = sizeof(sim_data);

	// 调试信息
	if (frame_counter % 25 == 0) {  // 每1秒打印一次
		PX4_INFO("闭环控制状态: offset=(%d,%d), control=(%.3f,%.3f), locked=%d",
				_current_offset_x, _current_offset_y,
				(double)_last_vx, (double)_last_vy,
				(int)target_locked);
	}
}
