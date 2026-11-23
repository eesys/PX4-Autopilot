# px4_add_module lives here on main. This copy still builds MAIN as an executable
# from SRCS so uorb_tests can run. main builds a static library named MODULE and
# parses arguments with px4_parse_function_args.

function(px4_add_module)
	cmake_parse_arguments(PX4 "" "MODULE;MAIN;PRIORITY" "SRCS" ${ARGN})
	add_executable(${PX4_MAIN} ${PX4_SRCS})
	target_include_directories(${PX4_MAIN} PRIVATE
		${CMAKE_SOURCE_DIR}/platforms/common
		${CMAKE_SOURCE_DIR}/platforms/common/include
		${CMAKE_BINARY_DIR}
	)
	# Debt: main gets these includes from px4_add_common_flags.
	# Topic headers are generated at ${CMAKE_BINARY_DIR}/uORB/topics.
	# WorkItem lives under platforms/common/include.
	if(TARGET uorb_headers)
		add_dependencies(${PX4_MAIN} uorb_headers)
	endif()
endfunction()
