cmake_minimum_required(VERSION 3.8)

project(PI_icm20948 VERSION 1.0 LANGUAGES C CXX)

set(CXX_STANDARD 17)
set(CXX_STANDARD_REQUIRED True)

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/Pi-ICM20948.cpp)
file(GLOB UTILS ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/*.c)
message("SOURCES =" ${SOURCES})
message("utils =" ${UTILS})

add_library(PI_icm20948 SHARED ${SOURCES} ${UTILS})

target_include_directories(PI_icm20948 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
