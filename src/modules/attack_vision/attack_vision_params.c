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
 * Legacy pixel-control gain (m/s per pixel).
 *
 * The current LOS guidance path keeps this parameter for compatibility, but
 * does not use it for the active velocity setpoint calculation.
 *
 * @group Attack Vision
 * @reboot_required false
 */
PARAM_DEFINE_FLOAT(AV_KP, 0.001f);

/**
 * Legacy pixel deadzone threshold.
 *
 * The current LOS guidance path keeps this parameter for compatibility, but
 * does not use it for the active velocity setpoint calculation.
 *
 * @group Attack Vision
 * @reboot_required false
 */
PARAM_DEFINE_FLOAT(AV_DEAD, 5.0f);

/**
 * Legacy pixel-control maximum velocity limit.
 *
 * The current LOS guidance path keeps this parameter for compatibility. Use
 * AV_FORWARD_V and AV_MAX_VZ for the active velocity limits.
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

/**
 * 图像宽度，用于把像素脱靶量换算为目标视线角
 * @group Attack Vision
 * @min 1
 * @reboot_required false
 */
PARAM_DEFINE_INT32(AV_CAM_W, 1920);

/**
 * 图像高度，用于把像素脱靶量换算为目标视线角
 * @group Attack Vision
 * @min 1
 * @reboot_required false
 */
PARAM_DEFINE_INT32(AV_CAM_H, 1080);

/**
 * 相机水平视场角，默认使用CGTD055宽视场
 * @group Attack Vision
 * @unit deg
 * @min 1
 * @max 179
 * @reboot_required false
 */
PARAM_DEFINE_FLOAT(AV_FOV_H, 70.4f);

/**
 * 相机垂直视场角，默认使用CGTD055宽视场
 * @group Attack Vision
 * @unit deg
 * @min 1
 * @max 179
 * @reboot_required false
 */
PARAM_DEFINE_FLOAT(AV_FOV_V, 39.6f);

/**
 * 像素Y方向符号转换
 *
 * 1: 脱靶量Y按笛卡尔坐标向上为正，转换到FRD坐标时取反
 * 0: 脱靶量Y按图像坐标向下为正，转换到FRD坐标时不取反
 *
 * @group Attack Vision
 * @min 0
 * @max 1
 * @reboot_required false
 */
PARAM_DEFINE_INT32(AV_PIX_Y_INV, 1);

/**
 * 末制导垂向速度限幅，NED坐标下向下为正
 * @group Attack Vision
 * @unit m/s
 * @min 0
 * @reboot_required false
 */
PARAM_DEFINE_FLOAT(AV_MAX_VZ, 0.8f);

/**
 * 外部任务协同模式
 *
 * 0: Standalone，忽略external_mission_active，仅由attack_vision单独末制导
 * 1: Coordinated，严格使用上位机external_mission_active协同状态
 * 2: Auto，收到上位机协同状态时使用Coordinated，否则退回Standalone
 *
 * @group Attack Vision
 * @min 0
 * @max 2
 * @reboot_required false
 */
PARAM_DEFINE_INT32(AV_EXT_MODE, 1);
