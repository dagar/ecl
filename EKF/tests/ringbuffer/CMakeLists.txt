
include_directories(../..)

add_executable(test_ringbuffer ringbuffer.cpp)
target_link_libraries(test_ringbuffer ekf)

add_test(NAME test_ringbuffer
	COMMAND test_ringbuffer)
