px4_add_module(
	MODULE modules__vision_guidance
	MAIN vision_guidance
	SRCS
		# VisionGuidance_gz.cpp
		VisionGuidance_realuart.cpp
		main.cpp
		vserial.cpp

	DEPENDS
		px4_work_queue
)
