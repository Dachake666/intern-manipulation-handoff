cmake_minimum_required(VERSION 3.8)
project(camera_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_interface REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${vision_interface_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(${PROJECT_NAME}
  src/cameraClient.cpp
  src/test_camera_client.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  vision_interface
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
