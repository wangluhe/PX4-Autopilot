/**
 * Attack Vision parameters
 */

#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

PARAM_DEFINE_INT32(AV_EN, 1);
PARAM_DEFINE_INT32(AV_BAUD, 115200);
PARAM_DEFINE_FLOAT(AV_KP, 0.001f);      // m/s per pixel
PARAM_DEFINE_FLOAT(AV_DEAD, 5.0f);      // pixels deadzone
PARAM_DEFINE_FLOAT(AV_MAX_V, 1.0f);     // m/s limit


