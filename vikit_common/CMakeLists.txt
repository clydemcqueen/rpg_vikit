cmake_minimum_required (VERSION 3.14)
project(vikit_common)
set(CMAKE_BUILD_TYPE RelWithDebInfo) # Release, RelWithDebInfo, Debug
set(USE_ROS TRUE)

# Set build flags. Set IS_ARM on odroid board as environment variable
set(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
if(DEFINED ENV{ARM_ARCHITECTURE})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)

include_directories(include)

if(USE_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
else()
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
endif()

# Create vikit library
add_library(vikit_common STATIC
  src/atan_camera.cpp
  src/omni_camera.cpp
  src/math_utils.cpp
  src/vision.cpp
  src/performance_monitor.cpp
  src/robust_cost.cpp
  src/user_input_thread.cpp
  src/pinhole_camera.cpp
  src/homography.cpp
  src/img_align.cpp)

if(USE_ROS)
  # Modern CMake target
  add_library(vikit_common::vikit_common ALIAS vikit_common)
endif()

target_link_libraries(vikit_common
  PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
    Sophus::Sophus)

# Tests
add_executable(test_vk_common_camera test/test_camera.cpp)
target_link_libraries(test_vk_common_camera vikit_common)

add_executable(test_vk_common_triangulation test/test_triangulation.cpp)
target_link_libraries(test_vk_common_triangulation vikit_common)

add_executable(test_vk_common_patch_score test/test_patch_score.cpp)
target_link_libraries(test_vk_common_patch_score vikit_common)

################################################################################
# Create the vikit_commonConfig.cmake file for other cmake projects.
if(USE_ROS)
  install(TARGETS vikit_common EXPORT vikit_common_targets INCLUDES DESTINATION include)
  install(DIRECTORY include/ DESTINATION include)  # Note trailing /
  ament_export_targets(vikit_common_targets)
  ament_export_dependencies(rclcpp)
  ament_package()
else()
  # Temporary hack:
  cmake_policy(SET CMP0026 OLD)

  get_target_property( FULL_LIBRARY_NAME vikit_common LOCATION )
  set(vikit_common_LIBRARIES ${FULL_LIBRARY_NAME} )
  set(vikit_common_LIBRARY_DIR ${PROJECT_BINARY_DIR} )
  set(vikit_common_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/vikit_commonConfig.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}/vikit_commonConfig.cmake @ONLY IMMEDIATE)
  export(PACKAGE vikit_common)

  install(DIRECTORY include/vikit DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.h")
  install(TARGETS vikit_common DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
endif()