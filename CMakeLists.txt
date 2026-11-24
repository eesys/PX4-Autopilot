cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(px4 CXX C ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PX4_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE FILEPATH "PX4 source directory" FORCE)
set(CMAKE_MODULE_PATH ${PX4_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
include(px4_add_module)

# Debt: main sets this from kconfig when TESTING is enabled.
set(PX4_TESTING "1" CACHE INTERNAL "testing enabled" FORCE)

add_subdirectory(msg EXCLUDE_FROM_ALL)

# Debt: main passes EXCLUDE_FROM_ALL. Left off so `cmake --build` builds uorb_tests.
add_subdirectory(platforms)

# Debt: main builds src from the board cmake.
add_subdirectory(src)

# Debt: main calls include(CTest) only when CMAKE_TESTING is on, and does not add_test this module.
include(CTest)
if(BUILD_TESTING)
	add_test(NAME uorb_tests COMMAND uorb_tests)
	add_test(NAME wqueue_test COMMAND wqueue_test)
	add_test(NAME simulator_sih COMMAND simulator_sih)
endif()
