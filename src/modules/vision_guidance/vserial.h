#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <drivers/drv_hrt.h>
#include <string.h>

class VSerial
{
	public:
	static VSerial& get_instance();

	int init();
	int write(const uint8_t *data, size_t length);
	int read(uint8_t *buffer, size_t length);
	bool is_initialized() const { return _initialized; }

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
};
