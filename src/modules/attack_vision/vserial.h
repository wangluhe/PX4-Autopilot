#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <random>

class VSerial
{
public:
	static VSerial& get_instance();

	int init();
	int write(const uint8_t *data, size_t length);
	int read(uint8_t *buffer, size_t length);
	bool is_initialized() const { return _initialized; }


	// 新增：获取当前脱靶量（用于调试）
	void get_target_offset(int16_t &x, int16_t &y) const;
	// 新增：设置控制量输入
	void set_control_input(float vx, float vy, float vz);

	// 新增：获取当前随机生成的吊舱姿态
	void get_gimbal_attitude(float &roll_deg, float &pitch_deg, float &yaw_deg) const;

private:
	VSerial() = default;
	~VSerial();

	bool _initialized{false};
	int _vserial_fd{-1};
	hrt_abstime _last_sim_time{0};

	// Simulation data generation
	void generate_sim_data();
	uint8_t _sim_buffer[64]{};
	size_t _sim_data_available{0};

	// 新增：闭环控制相关变量
	hrt_abstime _last_control_time{0};

	// 新增：闭环控制相关变量
	float _last_vx{0.0f};    // 前向速度
	float _last_vy{0.0f};    // 横向速度
	float _last_vz{0.0f};    // 垂直速度

	// 当前脱靶量状态
	int16_t _current_offset_x{0};  // 初始脱靶量
	int16_t _current_offset_y{200};

	// 新增：随机生成的吊舱姿态角度（单位：0.01度）
	int16_t _random_roll_deg100{0};
	int16_t _random_pitch_deg100{0};
	int16_t _random_yaw_deg100{0};

	// 新增：随机数生成器
	std::default_random_engine _random_engine;
	std::uniform_int_distribution<int16_t> _roll_distribution{-3000, 3000};   // -30° ~ 30°
	std::uniform_int_distribution<int16_t> _pitch_distribution{-3000, 3000};  // -30° ~ 30°
	std::uniform_int_distribution<int16_t> _yaw_distribution{-18000, 18000};       // 0° ~ 360°


	// 闭环控制参数
	static constexpr float CONTROL_GAIN = 0.1f;  // 控制增益
	static constexpr float DECAY_RATE = 0.95f;   // 自然衰减率
	static constexpr int16_t MAX_OFFSET = 200;   // 最大脱靶量

	// 新增：生成随机姿态角度
	void generate_random_attitude();
};
