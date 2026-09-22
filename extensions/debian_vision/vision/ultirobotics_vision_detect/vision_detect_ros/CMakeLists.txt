cmake_minimum_required(VERSION 3.8)
project(vision_detect_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(vision_interface REQUIRED)
find_package(vision_utils REQUIRED)
find_package(vision_algorithm REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(nlohmann REQUIRED)
find_package(spdlog REQUIRED)
find_package(onnxruntime REQUIRED)
find_package(fmt REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${vision_interface_INCLUDE_DIRS}
  ${vision_utils_INCLUDE_DIRS}
  ${vision_algorithm_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${nlohmann_INCLUDE_DIR}
  ${spdlog_INCLUDE_DIR}
  ${onnxruntime_INCLUDE_DIR}
  ${ZMQ_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

file(GLOB HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/visionDetectServer.cpp
  src/cameraClientManager.cpp
  ${HEADER_FILES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  vision_interface
  vision_algorithm
)

target_link_libraries(${PROJECT_NAME}
  ${vision_utils_LIBRARIES}
  ${vision_algorithm_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${onnxruntime_LIB}
  ${ZMQ_LIBRARIES}
  ${spdlog_LIB}
  fmt::fmt
  pthread
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
