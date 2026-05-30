cmake_minimum_required(VERSION 3.16)
project(RocketDeadReckoning C)

set(CMAKE_C_STANDARD 11)

# Host build for unit tests only (not for embedded target)
set(SOURCES
    nav/attitude.c
    nav/dead_reckoning.c
    nav/apogee_detector.c
    flight_fsm.c
)

add_library(rocket_nav STATIC ${SOURCES})
target_include_directories(rocket_nav PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(rocket_nav PRIVATE -Wall -Wextra -pedantic)

# Unit test binary
add_executable(test_dr tests/test_dead_reckoning.c)
target_link_libraries(test_dr rocket_nav m)
