############################################################################
#
#   Copyright (c) 2025.
#
#   Image terminal guidance module (attack_vision)
#
############################################################################

px4_add_module(
	MODULE modules__attack_vision
	MAIN attack_vision
	SRCS
		attack_vision_test_copy.cpp
		attack_vision_params.c
		vserial.cpp
	DEPENDS
		px4_work_queue
)


