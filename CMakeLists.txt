cmake_minimum_required(VERSION 3.8)
project(rvio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV not found.")
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/rvio/System.cc
  src/rvio/Updater.cc
  src/rvio/Tracker.cc
  src/rvio/PreIntegrator.cc
  src/rvio/Ransac.cc
  src/rvio/InputBuffer.cc
  src/rvio/FeatureDetector.cc
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2
  tf2_ros
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  visualization_msgs
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)
set(CMAKE_BUILD_RPATH "$ORIGIN;$ORIGIN/..")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/..")
# Create executable
add_executable(rvio_mono src/rvio_mono.cc)
ament_target_dependencies(rvio_mono
  rclcpp
  tf2
  tf2_ros
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
)

target_link_libraries(rvio_mono
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${PROJECT_NAME}
  tcmalloc_and_profiler
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)
install(TARGETS
  rvio_mono
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
