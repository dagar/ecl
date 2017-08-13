
include_directories(../..)

add_executable(test_base base.cpp)
target_link_libraries(test_base ekf geo eclmath)

add_test(NAME test_base
	COMMAND test_base)