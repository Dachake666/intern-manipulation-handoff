cmake_minimum_required(VERSION 3.5)
project(camera_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_interface REQUIRED)
find_package(vision_utils REQUIRED)
find_package(OpenCV REQUIRED)
find_package(obCamera REQUIRED)
find_package(tyCamera REQUIRED)
find_package(nlohmann REQUIRED)
find_package(spdlog REQUIRED)
find_package(camera_drive REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${vision_interface_INCLUDE_DIRS}
  ${vision_utils_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${obCamera_INCLUDE_DIR}
  ${tyCamera_INCLUDE_DIR}
  ${nlohmann_INCLUDE_DIR}
  ${spdlog_INCLUDE_DIR}
  ${camera_drive_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

file(GLOB HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

add_executable(${PROJECT_NAME} 
  src/main.cpp
  src/cameraServer.cpp
  ${HEADER_FILES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  vision_interface
)

target_link_libraries(${PROJECT_NAME}
  ${vision_utils_LIBRARIES}
  ${OpenCV_LIBS} 
  pthread
  ${tyCamera_LIB}
  ${obCamera_LIB}
  fmt 
  ${spdlog_LIB}
  ${camera_drive_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
