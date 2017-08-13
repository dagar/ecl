
set(SRC
	airspeed_fusion.cpp
	control.cpp
	covariance.cpp
	drag_fusion.cpp
	ekf.cpp
	ekf_helper.cpp
	estimator_interface.cpp
	gps_checks.cpp
	mag_fusion.cpp
	optflow_fusion.cpp
	sideslip_fusion.cpp
	terrain_estimator.cpp
	vel_pos_fusion.cpp
	)
	
add_library(ekf ${SRC})
