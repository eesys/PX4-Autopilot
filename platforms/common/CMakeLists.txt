if(NOT "${PX4_BOARD}" MATCHES "io-v2")
	add_subdirectory(uORB)
endif()

add_subdirectory(px4_work_queue)
