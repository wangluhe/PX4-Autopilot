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
 * Attack Vision module enable
 *
 * @group Attack Vision
 * @reboot_required false
 */

PARAM_DEFINE_INT32(AAATTKVIS_EN, 1);

/**
 * UART baudrate for gimbal communication
 *
 * @group Attack Vision
 * @reboot_required false
 */
PARAM_DEFINE_INT32(AV_BAUD, 115200);

/**
 * Velocity control gain (m/s per pixel)
 *
 * @group Attack Vision
 * @reboot_required false
 */
PARAM_DEFINE_FLOAT(AV_KP, 0.001f);

/**
 * Pixel deadzone threshold
 *
 * @group Attack Vision
 * @reboot_required false
 */
PARAM_DEFINE_FLOAT(AV_DEAD, 5.0f);

/**
 * Maximum velocity limit (m/s)
 *
 * @group Attack Vision
 * @reboot_required false
 */
PARAM_DEFINE_FLOAT(AV_MAX_V, 1.0f);


// 在attack_vision_params.c中添加新参数
/**
 * 姿态控制增益
 * @group Attack Vision
 * @reboot_required false
 */
PARAM_DEFINE_FLOAT(AV_ATT_KP, 0.1f);

/**
 * 最大接近速度 (m/s)
 * @group Attack Vision
 * @reboot_required false
 */
PARAM_DEFINE_FLOAT(AV_APPROACH_V, 2.0f);

/**
 * 控制策略选择 (0=像素控制, 1=姿态控制, 2=混合控制)
 * @group Attack Vision
 * @reboot_required false
 */
PARAM_DEFINE_INT32(AV_STRATEGY, 2);

/**
 * 前向接近速度 (m/s)
 * @group Attack Vision
 * @reboot_required false
 */
PARAM_DEFINE_FLOAT(AV_FORWARD_V, 1.0f);




