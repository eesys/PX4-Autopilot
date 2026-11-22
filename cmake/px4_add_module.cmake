# px4_add_module lives here on main. This copy still builds MAIN as an executable
# from SRCS so uorb_tests can run. main builds a static library named MODULE and
# parses arguments with px4_parse_function_args.

function(px4_add_module)
	cmake_parse_arguments(PX4 "" "MODULE;MAIN;PRIORITY" "SRCS" ${ARGN})
	add_executable(${PX4_MAIN} ${PX4_SRCS})
	target_include_directories(${PX4_MAIN} PRIVATE ${CMAKE_SOURCE_DIR}/platforms/common)
endfunction()
