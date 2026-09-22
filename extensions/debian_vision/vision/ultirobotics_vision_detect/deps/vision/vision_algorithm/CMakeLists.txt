cmake_minimum_required(VERSION 3.8)
project(vision_algorithm)

find_package(ament_cmake REQUIRED)

# colcon build 时 CMAKE_BINARY_DIR = ${workspace}/build/${package}
# 所以 workspace = dirname(CMAKE_BINARY_DIR)
get_filename_component(WORKSPACE_DIR "${CMAKE_BINARY_DIR}" PATH)

set(REAL_BUILD_DIR "${CMAKE_BINARY_DIR}/real_build")
set(SO_FILE "${REAL_BUILD_DIR}/libvision_algorithm.so")

# 查找所有源文件和头文件作为依赖
file(GLOB_RECURSE SOURCE_FILES 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)
file(GLOB_RECURSE HEADER_FILES 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp
)

set(CMAKE_BUILD_FILE ${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt.build)

add_custom_command(
  OUTPUT ${SO_FILE}
  COMMAND ${CMAKE_COMMAND} -E env 
    WORKSPACE_DIR=${WORKSPACE_DIR}
    REAL_BUILD_DIR=${REAL_BUILD_DIR}
    SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    bash ${CMAKE_CURRENT_SOURCE_DIR}/build_vision.sh
  DEPENDS ${SOURCE_FILES} ${HEADER_FILES} ${CMAKE_BUILD_FILE}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Building vision_algorithm library"
  VERBATIM
)

add_custom_target(vision_algorithm_lib ALL
  DEPENDS ${SO_FILE}
)

install(FILES
  ${SO_FILE}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
  include/image_processing
  include/point_cloud_processing
  include/sam3
  include/yolo
)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(OpenCV PCL Eigen3)

ament_package()
