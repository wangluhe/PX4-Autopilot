#include <px4_platform_common/module.h>
#include "VisionGuidance.hpp"

extern "C" __EXPORT int vision_guidance_main(int argc, char *argv[]);

int vision_guidance_main(int argc, char *argv[])
{
	return VisionGuidance::main(argc, argv);
}
