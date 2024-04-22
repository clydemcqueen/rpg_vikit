set(PROJECT_NAME vikit_ros)
cmake_minimum_required (VERSION 3.14)
project(${PROJECT_NAME})
set(CMAKE_BUILD_TYPE RelWithDebInfo) # Release, RelWithDebInfo, Debug

# Set build flags
set(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)

# Create vikit library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/output_helper.cpp)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${OpenCV_LIBS}
  Sophus::Sophus)

ament_auto_add_executable(test_camera
  src/test_camera.cpp)

ament_auto_package()
