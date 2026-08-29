cmake_minimum_required(VERSION 3.16)
project(jaka_mini2_orbbec_camera LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(ORBBEC_SDK_ROOT "" CACHE PATH "Path to the unpacked Orbbec SDK")
if(NOT EXISTS "${ORBBEC_SDK_ROOT}/include/libobsensor/ObSensor.hpp")
  message(FATAL_ERROR "Set ORBBEC_SDK_ROOT to an unpacked Orbbec SDK")
endif()

add_library(orbbec_dual_camera SHARED orbbec_camera_c_api.cpp)
target_include_directories(orbbec_dual_camera PRIVATE "${ORBBEC_SDK_ROOT}/include")
target_link_directories(orbbec_dual_camera PRIVATE "${ORBBEC_SDK_ROOT}/lib")
target_link_libraries(orbbec_dual_camera PRIVATE OrbbecSDK pthread)
set_target_properties(orbbec_dual_camera PROPERTIES BUILD_RPATH "${ORBBEC_SDK_ROOT}/lib")

add_executable(orbbec_color_smoke orbbec_color_smoke.cpp)
target_include_directories(orbbec_color_smoke PRIVATE "${ORBBEC_SDK_ROOT}/include")
target_link_directories(orbbec_color_smoke PRIVATE "${ORBBEC_SDK_ROOT}/lib")
target_link_libraries(orbbec_color_smoke PRIVATE OrbbecSDK pthread)
set_target_properties(orbbec_color_smoke PROPERTIES BUILD_RPATH "${ORBBEC_SDK_ROOT}/lib")
