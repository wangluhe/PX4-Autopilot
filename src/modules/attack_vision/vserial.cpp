#include "vserial.h"
#include <px4_platform_common/log.h>
#include <cmath>

VSerial& VSerial::get_instance()
{
	static VSerial instance;
	return instance;
}

VSerial::~VSerial()
{
    // 清理资源（如果有的话）
}

int VSerial::init()
{
	if (_initialized) {
		return 0;
	}

	// 在SITL模式下，我们模拟一个虚拟串口
	_initialized = true;
	_last_sim_time = hrt_absolute_time();

	PX4_INFO("Virtual serial port initialized for SITL simulation");
	return 0;
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

	// 生成模拟数据
	generate_sim_data();

	if (_sim_data_available > 0) {
		size_t copy_size = _sim_data_available;
		if (copy_size > length) {
		copy_size = length;
		}

		memcpy(buffer, _sim_buffer, copy_size);
		_sim_data_available = 0;

		PX4_DEBUG("VSerial read: %zu bytes", copy_size);
		return copy_size;
	}

	return 0;
	}

// void VSerial::generate_sim_data()
// 	{
// 	hrt_abstime now = hrt_absolute_time();

// 	// 每40ms生成一次数据（25Hz）
// 	if (now - _last_sim_time < 40000) {
// 		return;
// 	}

// 	_last_sim_time = now;

// 	// 模拟视觉吊舱数据包结构
// 	struct SimPodData {
// 		uint8_t header1;        // 0xFC
// 		uint8_t header2;        // 0x2C
// 		uint8_t payload_type;   // 载荷类型
// 		uint8_t self_test;      // 自检结果
// 		uint16_t status1;       // 吊舱状态1
// 		uint16_t status2;       // 吊舱状态2
// 		uint8_t servo_status;   // 伺服状态
// 		int16_t azimuth;        // 方位角 ×100
// 		int16_t elevation;      // 俯仰角 ×100
// 		int16_t roll;           // 横滚角 ×100
// 		uint8_t reserved1[3];   // 备用
// 		uint8_t target_info;    // 目标编号及类型
// 		uint8_t reserved2[2];   // 备用
// 		uint8_t tf_usage;       // TF使用容量百分比
// 		uint8_t tf_total;       // TF卡总容量
// 		uint16_t ir_focus;      // 红外焦距 ×10
// 		uint16_t visible_focus; // 可见光焦距 ×10
// 		uint16_t target_id;     // 目标识别编号
// 		float target_lon;       // 目标经度
// 		float target_lat;       // 目标纬度
// 		int16_t target_alt;     // 目标海拔高度
// 		int16_t soc_temp;       // SOC处理器温度
// 		int16_t gyro_azimuth;   // 陀螺方位角速度 ×100
// 		int16_t gyro_elevation; // 陀螺俯仰角速度 ×100
// 		int16_t gyro_roll;      // 陀螺横滚角速度 ×100
// 		uint8_t display_mode;   // 当前显示图像反馈
// 		uint8_t reserved3[2];   // 备用
// 		int16_t track_point_x;  // 目标跟踪点宽方向坐标
// 		int16_t track_point_y;  // 目标跟踪点高方向坐标
// 		int16_t track_box_w;    // 目标跟踪框宽方向长度
// 		int16_t track_box_h;    // 目标跟踪框高方向长度
// 		int16_t off_target_azimuth;  // 目标脱靶量方位方向
// 		int16_t off_target_elevation;// 目标脱靶量俯仰方向
// 		uint8_t checksum;       // 异或校验
// 		uint8_t footer;         // 0xF0
// 	} __attribute__((packed));


// 	static SimPodData sim_data{};
// 	static uint32_t frame_counter = 0;
// 	static bool target_locked = false;

// 	frame_counter++;

// 	// 模拟目标锁定状态切换（每100帧切换一次）
// 	if (frame_counter % 100 == 0) {
// 		target_locked = !target_locked;
// 		PX4_INFO("Simulation: Target %s", target_locked ? "LOCKED" : "LOST");
// 	}
// 	// PX4_INFO("Generated sim data - Status1: 0x%04X, Target locked: %s", sim_data.status1, target_locked ? "YES" : "NO");


// 	// 填充数据包
// 	sim_data.header1 = 0xFC;
// 	sim_data.header2 = 0x2C;
// 	sim_data.payload_type = 0x01;
// 	sim_data.self_test = 0xFF;  // 自检正常

// 	// 设置状态：如果目标锁定，设置相应的位
// 	// if (target_locked) {
// 	// 	sim_data.status1 = (1 << 9);  // 设置锁定状态位
// 	// } else {
// 	// 	sim_data.status1 = 0;
// 	// }

// 	if (target_locked) {
// 		sim_data.status1 = (1 << 9);  // 设置锁定状态位
// 		sim_data.servo_status = 0x07; // 跟踪模式
// 		PX4_DEBUG("生成锁定状态数据: status1=0x%04X", sim_data.status1);
// 	} else {
// 		sim_data.status1 = 0;
// 		sim_data.servo_status = 0x00; // 非跟踪模式
// 	}

// 	sim_data.status2 = 0;
// 	// sim_data.servo_status = 0xFF;

// 	// 模拟角度数据
// 	sim_data.azimuth = 1000;    // 10.00度
// 	sim_data.elevation = -500;  // -5.00度
// 	sim_data.roll = 0;

// 	// 模拟目标信息
// 	sim_data.target_info = target_locked ? 0x01 : 0x00;

// 	// 模拟脱靶量 - 在锁定状态下产生小的脱靶量
// 	if (target_locked) {
// 		// 产生正弦波形的脱靶量，模拟目标移动
// 		float time_sec = now / 1e6f;
// 		sim_data.off_target_azimuth = static_cast<int16_t>(50.0f * sinf(time_sec * 2.0f));
// 		sim_data.off_target_elevation = static_cast<int16_t>(30.0f * sinf(time_sec * 1.5f));
// 	} else {
// 		sim_data.off_target_azimuth = 0;
// 		sim_data.off_target_elevation = 0;
// 	}

// 	// 计算校验和
// 	sim_data.checksum = 0;
// 	const uint8_t *data_ptr = reinterpret_cast<const uint8_t*>(&sim_data);
// 	for (int i = 2; i < 62; i++) {
// 		sim_data.checksum ^= data_ptr[i];
// 	}

// 	sim_data.footer = 0xF0;

// 	// 复制到输出缓冲区
// 	memcpy(_sim_buffer, &sim_data, sizeof(sim_data));
// 	_sim_data_available = sizeof(sim_data);
// }

void VSerial::generate_sim_data()
{
	hrt_abstime now = hrt_absolute_time();

	// 每40ms生成一次数据（25Hz）
	if (now - _last_sim_time < 40000) {
		return;
	}

	_last_sim_time = now;

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
	static bool target_locked = true;

	frame_counter++;

	// 模拟目标锁定状态切换（每100帧切换一次）
	if (frame_counter % 500 == 0) {
		target_locked = !target_locked;
		PX4_INFO("Simulation: Target %s", target_locked ? "LOCKED" : "LOST");
	}

	// 填充数据包
	sim_data.header1 = 0xFC;
	sim_data.header2 = 0x2C;
	sim_data.payload_type = 0x01;
	sim_data.self_test = 0xFF;  // 自检正常

	// 设置状态：如果目标锁定，设置相应的位
	if (target_locked) {
		sim_data.status1 = (1 << 9);  // 设置锁定状态位
		sim_data.servo_status = 0x07; // 跟踪模式
		PX4_DEBUG("生成锁定状态数据: status1=0x%04X, servo_status=0x%02X",
			sim_data.status1, sim_data.servo_status);
	} else {
		sim_data.status1 = 0;
		sim_data.servo_status = 0x00; // 非跟踪模式
	}

	sim_data.status2 = 0;
	// 注意：这里删除了重复设置 servo_status = 0xFF 的代码

	// 模拟角度数据
	sim_data.azimuth = 1000;    // 10.00度
	sim_data.elevation = -500;  // -5.00度
	sim_data.roll = 0;

	// 模拟目标信息
	sim_data.target_info = target_locked ? 0x01 : 0x00;

	// 模拟脱靶量 - 在锁定状态下产生小的脱靶量
	if (target_locked) {
		// 产生正弦波形的脱靶量，模拟目标移动
		// float time_sec = now / 1e6f;
		// sim_data.off_target_azimuth = static_cast<int16_t>(50.0f * sinf(time_sec * 2.0f));
		// sim_data.off_target_elevation = static_cast<int16_t>(30.0f * sinf(time_sec * 1.5f));
		sim_data.off_target_azimuth = static_cast<int16_t>(50.0f);
		sim_data.off_target_elevation = static_cast<int16_t>(30.0f);
	} else {
		sim_data.off_target_azimuth = 0;
		sim_data.off_target_elevation = 0;
	}

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
}
