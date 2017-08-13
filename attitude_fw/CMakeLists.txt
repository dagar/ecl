
set(SRC
	ecl_controller.cpp
	ecl_pitch_controller.cpp
	ecl_roll_controller.cpp
	ecl_wheel_controller.cpp
	ecl_yaw_controller.cpp
	)
	
add_library(attitude_fw ${SRC})
