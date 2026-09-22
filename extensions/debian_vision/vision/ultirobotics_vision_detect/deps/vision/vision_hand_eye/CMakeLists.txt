cmake_minimum_required(VERSION 3.8)
project(vision_hand_eye)

# 编译选项：开启警告、C++14 标准（兼容多数第三方库）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /opt/ros/humble)
# ======================================
# 1. 查找 ROS 2 依赖包（必须正确配置，否则找不到接口头文件）
# ======================================
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_interface REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS           # 点云处理（3D 检测依赖）
  common
  io
  filters
  segmentation
)
find_package(nlohmann REQUIRED)

include_directories(${rclcpp_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${visition_action_interfaces} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${nlohmann_INCLUDE_DIR})

#eigen
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/include/eigen3 ${CMAKE_CURRENT_SOURCE_DIR}/include)
file(GLOB HEADER_FILES ${PROJECT_SOURCE_DIR}/include/*.h)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

#add_executable(${PROJECT_NAME} src/test.cpp src/cameraClient.cpp src/calibration_server.cpp src/camera_calibration.cpp src/markdetector.cpp src/local_data_test.cpp ${HEADER_FILES})
#target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} -lpthread fmt spdlog ${PCL_LIBRARIES})
#ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs vision_interface)

# Add the official node executable
add_executable(${PROJECT_NAME} src/hand_eye_calibration_node.cpp src/cameraClient.cpp src/calibration_server.cpp src/camera_calibration.cpp src/markdetector.cpp src/local_data_test.cpp ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} -lpthread fmt spdlog ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs vision_interface)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# 声明包信息（必须，否则 colcon 无法识别该包）
ament_package()
