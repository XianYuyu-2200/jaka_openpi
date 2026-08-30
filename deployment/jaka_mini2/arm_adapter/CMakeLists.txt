cmake_minimum_required(VERSION 3.16)
project(jaka_mini2_adapter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

add_library(jaka_adapter_headers INTERFACE)
target_include_directories(jaka_adapter_headers INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(dry_run_core_test tests/dry_run_core_test.cpp)
target_link_libraries(dry_run_core_test PRIVATE jaka_adapter_headers)
add_test(NAME dry_run_core COMMAND dry_run_core_test)

add_executable(waypoint_recorder_core_test tests/waypoint_recorder_core_test.cpp)
target_link_libraries(waypoint_recorder_core_test PRIVATE jaka_adapter_headers)
foreach(mode stable joint_jitter tcp_jitter read_failure yaml_policy)
  add_test(NAME waypoint_${mode} COMMAND waypoint_recorder_core_test ${mode})
endforeach()

option(BUILD_JAKA_HARDWARE_TOOLS "Build programs that link to the proprietary JAKA SDK" OFF)
set(JAKA_SDK_ROOT "" CACHE PATH "Directory containing include/jaka_driver and lib/libjakaAPI.so")

if(BUILD_JAKA_HARDWARE_TOOLS)
  if(NOT EXISTS "${JAKA_SDK_ROOT}/include/jaka_driver/JAKAZuRobot.h")
    message(FATAL_ERROR "Set JAKA_SDK_ROOT to jaka_ros2/src/jaka_driver")
  endif()

  add_library(jaka_vendor_sdk INTERFACE)
  target_include_directories(jaka_vendor_sdk INTERFACE ${JAKA_SDK_ROOT}/include/jaka_driver)
  target_link_directories(jaka_vendor_sdk INTERFACE ${JAKA_SDK_ROOT}/lib)
  target_link_libraries(jaka_vendor_sdk INTERFACE jakaAPI pthread)

  foreach(tool operator_joint_publisher six_joint_follow_test waypoint_recorder o6_fc16_single_step)
    add_executable(${tool} src/${tool}.cpp)
    target_link_libraries(${tool} PRIVATE jaka_adapter_headers jaka_vendor_sdk)
    set_target_properties(${tool} PROPERTIES BUILD_RPATH ${JAKA_SDK_ROOT}/lib)
  endforeach()

  add_library(jaka_arm_backend src/jaka_arm_backend.cpp)
  target_link_libraries(jaka_arm_backend PRIVATE jaka_vendor_sdk)
  target_include_directories(jaka_arm_backend PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  set_target_properties(jaka_arm_backend PROPERTIES POSITION_INDEPENDENT_CODE ON)

  add_library(jaka_arm_c_api SHARED src/jaka_arm_c_api.cpp)
  target_link_libraries(jaka_arm_c_api PRIVATE jaka_arm_backend jaka_vendor_sdk)
  target_include_directories(jaka_arm_c_api PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  set_target_properties(jaka_arm_c_api PROPERTIES BUILD_RPATH ${JAKA_SDK_ROOT}/lib)
endif()
