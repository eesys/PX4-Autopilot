cmake_minimum_required(VERSION 3.16)
project(px4_from_scratch LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(uorb_test test/uorb_test.cpp)
target_include_directories(uorb_test PRIVATE ${CMAKE_SOURCE_DIR}/src)

enable_testing()
add_test(NAME uorb_test COMMAND uorb_test)
