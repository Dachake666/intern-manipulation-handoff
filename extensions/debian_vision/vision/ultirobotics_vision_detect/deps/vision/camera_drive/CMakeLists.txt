cmake_minimum_required(VERSION 3.5)
project(camera_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_utils REQUIRED)
find_package(OpenCV REQUIRED)
find_package(obCamera REQUIRED)
find_package(tyCamera REQUIRED)
find_package(nlohmann REQUIRED)
find_package(spdlog REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${vision_utils_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${obCamera_INCLUDE_DIR}
  ${tyCamera_INCLUDE_DIR}
  ${nlohmann_INCLUDE_DIR}
  ${spdlog_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ob_cam
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ty_cam
)

file(GLOB HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
                       ${CMAKE_CURRENT_SOURCE_DIR}/include/ob_cam/*.h
                       ${CMAKE_CURRENT_SOURCE_DIR}/include/ty_cam/*.h)

add_library(${PROJECT_NAME} SHARED
  src/ob_cam/obCamera.cpp 
  src/ty_cam/tyCam.cpp 
  src/ty_cam/TYThread.cpp
  src/camera_rgbd.cpp
  ${HEADER_FILES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

target_link_libraries(${PROJECT_NAME}
  ${vision_utils_LIBRARIES}
  ${OpenCV_LIBS} 
  pthread
  ${tyCamera_LIB}
  ${obCamera_LIB}
  fmt 
  ${spdlog_LIB}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  vision_utils
  OpenCV
)

ament_package()
