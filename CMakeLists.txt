################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(avoid_jetson)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

set(SLLIDAR_SDK_PATH "./sdk/")

FILE(GLOB SLLIDAR_SDK_SRC 
  "${SLLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
################################################################################
# Build
################################################################################
include_directories(
  ${SLLIDAR_SDK_PATH}/include
  ${SLLIDAR_SDK_PATH}/src
  ${Boost_INCLUDE_DIRS}
  include
)
include_directories(/usr/local/include/dynamixel_sdk)

add_executable(sllidar_node	src/sllidar_node.cpp ${SLLIDAR_SDK_SRC})
target_link_libraries(sllidar_node ${ament_cmake_LIBRARIES})
ament_target_dependencies(sllidar_node
  rclcpp
  std_srvs
  std_msgs
  sensor_msgs
)
add_executable(jetson src/jetson.cpp src/jetson_main.cpp src/dxl.cpp)
ament_target_dependencies(jetson rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(jetson dxl_x64_cpp)
# add_executable(lidarsave src/lidarsave.cpp src/main.cpp)
# ament_target_dependencies(lidarsave
#   rclcpp
#   std_srvs
#   sensor_msgs
#   OpenCV
# )
################################################################################
# Install
################################################################################

install(
  TARGETS 
  sllidar_node 
  jetson
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(OpenCV)
ament_export_include_directories(include)
ament_package()
