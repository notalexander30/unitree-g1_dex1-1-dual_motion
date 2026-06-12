cmake_minimum_required(VERSION 3.16)
project(unitree_g1_dual_dex1_motion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT DEFINED UNITREE_SDK2_ROOT AND DEFINED ENV{UNITREE_SDK2_ROOT})
  set(UNITREE_SDK2_ROOT "$ENV{UNITREE_SDK2_ROOT}")
endif()

if(NOT UNITREE_SDK2_ROOT)
  message(FATAL_ERROR "Set UNITREE_SDK2_ROOT to your local unitree_sdk2 checkout.")
endif()

get_filename_component(UNITREE_SDK2_ROOT "${UNITREE_SDK2_ROOT}" ABSOLUTE)
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable SDK examples for this wrapper build" FORCE)

add_subdirectory("${UNITREE_SDK2_ROOT}" "${CMAKE_BINARY_DIR}/unitree_sdk2")

add_executable(g1_dual_dex1_arm_hand_motion
  src/g1_dual_dex1_arm_hand_motion.cpp
)
target_link_libraries(g1_dual_dex1_arm_hand_motion PRIVATE unitree_sdk2)
