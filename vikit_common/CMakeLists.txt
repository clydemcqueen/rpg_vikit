set(PROJECT_NAME vikit_common)
cmake_minimum_required (VERSION 3.14)
project(${PROJECT_NAME})
set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
set(USE_ROS FALSE) # Set False if you want to build this package without Catkin

# Set build flags. Set IS_ARM on odroid board as environment variable
set(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
if(DEFINED ENV{ARM_ARCHITECTURE})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
find_package(Sophus REQUIRED)

include_directories(include)

if(USE_ROS)
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp cmake_modules)
  LIST(APPEND INCLUDE_DIRECTORIES ${catkin_INCLUDE_DIRS})
  catkin_package(
    DEPENDS Eigen OpenCV Sophus
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
  )
else()
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
endif()

# Create vikit library
add_library(${PROJECT_NAME} SHARED
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

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  Sophus::Sophus)

IF(USE_ROS)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${catkin_LIBRARIES})
ENDIF()

# Tests
add_executable(test_vk_common_camera test/test_camera.cpp)
target_link_libraries(test_vk_common_camera ${PROJECT_NAME})

add_executable(test_vk_common_triangulation test/test_triangulation.cpp)
target_link_libraries(test_vk_common_triangulation ${PROJECT_NAME})

add_executable(test_vk_common_patch_score test/test_patch_score.cpp)
target_link_libraries(test_vk_common_patch_score ${PROJECT_NAME})

################################################################################
# Create the vikit_commonConfig.cmake file for other cmake projects.
if(NOT USE_ROS)
  # Temporary hack:
  cmake_policy(SET CMP0026 OLD)

  get_target_property( FULL_LIBRARY_NAME ${PROJECT_NAME} LOCATION )
  set(vikit_common_LIBRARIES ${FULL_LIBRARY_NAME} )
  set(vikit_common_LIBRARY_DIR ${PROJECT_BINARY_DIR} )
  set(vikit_common_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/vikit_commonConfig.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}/vikit_commonConfig.cmake @ONLY IMMEDIATE)
  export(PACKAGE vikit_common)

  install(DIRECTORY include/vikit DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.h")
  install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
endif()