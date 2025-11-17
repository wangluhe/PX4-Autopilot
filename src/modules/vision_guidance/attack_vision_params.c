#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

// Define PARAM_DEFINE macros as empty (parameters are already in px4::parameters array)
#ifndef PARAM_DEFINE_INT32
#define PARAM_DEFINE_INT32(name, value) /* empty */
#endif
#ifndef PARAM_DEFINE_FLOAT
#define PARAM_DEFINE_FLOAT(name, value) /* empty */
#endif

/**
 * Vision Guidance azimuth control gain (x10000)
 *
 * @min 0
 * @max 10000
 * @group Vision Guidance
 */
PARAM_DEFINE_INT32(VISION_G_KP_AZ, 100);

/**
 * Vision Guidance elevation control gain (x10000)
 *
 * @min 0
 * @max 10000
 * @group Vision Guidance
 */
PARAM_DEFINE_INT32(VISION_G_KP_EL, 100);

/**
 * Vision Guidance maximum velocity (cm/s)
 *
 * @min 50
 * @max 500
 * @group Vision Guidance
 */
PARAM_DEFINE_INT32(VISION_G_MAX_VEL, 200);

/**
 * Vision Guidance UART port
 *
 * @value 0 /dev/ttyS0
 * @value 1 /dev/ttyS1
 * @value 2 /dev/ttyS2
 * @value 3 /dev/ttyS3
 * @value 4 /dev/ttyS4
 * @value 5 /dev/ttyS5
 * @value 6 /dev/ttyS6
 * @group Vision Guidance
 */
PARAM_DEFINE_INT32(VISION_UART_PORT, 2);

/**
 * Vision Guidance baudrate
 *
 * @value 9600 9600
 * @value 19200 19200
 * @value 38400 38400
 * @value 57600 57600
 * @value 115200 115200
 * @value 230400 230400
 * @value 460800 460800
 * @value 921600 921600
 * @group Vision Guidance
 */
PARAM_DEFINE_INT32(VISION_BAUDRATE, 115200);

