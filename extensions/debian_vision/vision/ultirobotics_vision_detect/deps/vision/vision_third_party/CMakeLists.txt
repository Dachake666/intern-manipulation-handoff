cmake_minimum_required(VERSION 3.8)
project(vision_third_party)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ===================== 检测系统架构 =====================
execute_process(
  COMMAND uname -m
  OUTPUT_VARIABLE SYSTEM_ARCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "[vision_third_party] Detected system architecture: ${SYSTEM_ARCH}")

# 根据架构设置变量
if(SYSTEM_ARCH MATCHES "x86_64|amd64|i686")
  set(ARCH_TYPE "x64")
  set(TYCAMERA_ARCH_DIR "lib_x64")
elseif(SYSTEM_ARCH MATCHES "aarch64")
  set(ARCH_TYPE "arm")
  set(TYCAMERA_ARCH_DIR "lib_Aarch64")
elseif(SYSTEM_ARCH MATCHES "armv7l|armv7hf")
  set(ARCH_TYPE "arm")
  set(TYCAMERA_ARCH_DIR "lib_armv7hf")
else()
  message(FATAL_ERROR "[vision_third_party] Unsupported architecture: ${SYSTEM_ARCH}")
endif()

message(STATUS "[vision_third_party] Using library directory: ${ARCH_TYPE}")
message(STATUS "[vision_third_party] tyCamera using: ${TYCAMERA_ARCH_DIR}")

# ===================== 定义路径 =====================
set(VISION_THIRD_PARTY_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(THIRD_PARTY_DIR "${VISION_THIRD_PARTY_ROOT}/third_party")

# -------------------- 校验目录存在性 --------------------
# nlohmann（纯头文件）
set(NLOHMANN_DIR "${THIRD_PARTY_DIR}/nlohmann")
if(NOT EXISTS "${NLOHMANN_DIR}")
  message(FATAL_ERROR "[vision_third_party] nlohmann目录不存在: ${NLOHMANN_DIR}")
endif()

# obCamera
set(OBCAMERA_INC_DIR "${THIRD_PARTY_DIR}/obCamera/include")
set(OBCAMERA_LIB_DIR "${THIRD_PARTY_DIR}/obCamera/lib/${ARCH_TYPE}")
if(NOT EXISTS "${OBCAMERA_INC_DIR}")
  message(FATAL_ERROR "[vision_third_party] obCamera头文件目录不存在: ${OBCAMERA_INC_DIR}")
endif()
if(NOT EXISTS "${OBCAMERA_LIB_DIR}")
  message(FATAL_ERROR "[vision_third_party] obCamera库目录不存在: ${OBCAMERA_LIB_DIR}")
endif()

# onnxruntime
set(ONNXRUNTIME_INC_DIR "${THIRD_PARTY_DIR}/onnxruntime/include")
set(ONNXRUNTIME_LIB_DIR "${THIRD_PARTY_DIR}/onnxruntime/lib/${ARCH_TYPE}")
if(NOT EXISTS "${ONNXRUNTIME_INC_DIR}")
  message(FATAL_ERROR "[vision_third_party] onnxruntime头文件目录不存在: ${ONNXRUNTIME_INC_DIR}")
endif()
if(NOT EXISTS "${ONNXRUNTIME_LIB_DIR}")
  message(FATAL_ERROR "[vision_third_party] onnxruntime库目录不存在: ${ONNXRUNTIME_LIB_DIR}")
endif()

# vision_spdlog
set(vision_spdlog_INC_DIR "${THIRD_PARTY_DIR}/vision_spdlog/include")
set(vision_spdlog_LIB_DIR "${THIRD_PARTY_DIR}/vision_spdlog/lib/${ARCH_TYPE}")
if(NOT EXISTS "${vision_spdlog_INC_DIR}")
  message(FATAL_ERROR "[vision_third_party] vision_spdlog头文件目录不存在: ${vision_spdlog_INC_DIR}")
endif()
if(NOT EXISTS "${vision_spdlog_LIB_DIR}")
  message(FATAL_ERROR "[vision_third_party] vision_spdlog库目录不存在: ${vision_spdlog_LIB_DIR}")
endif()

# tyCamera（保持原有结构）
set(TYCAMERA_INC_DIR "${THIRD_PARTY_DIR}/tyCamera/include")
set(TYCAMERA_LIB_DIR "${THIRD_PARTY_DIR}/tyCamera/lib/${TYCAMERA_ARCH_DIR}")
if(NOT EXISTS "${TYCAMERA_INC_DIR}")
  message(FATAL_ERROR "[vision_third_party] tyCamera头文件目录不存在: ${TYCAMERA_INC_DIR}")
endif()
if(NOT EXISTS "${TYCAMERA_LIB_DIR}")
  message(FATAL_ERROR "[vision_third_party] tyCamera库目录不存在: ${TYCAMERA_LIB_DIR}")
endif()

# -------------------- 安装头文件 --------------------
install(DIRECTORY ${NLOHMANN_DIR}
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
        )
install(DIRECTORY ${OBCAMERA_INC_DIR}/
        DESTINATION include/obCamera
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
        )
install(DIRECTORY ${ONNXRUNTIME_INC_DIR}/
        DESTINATION include/onnxruntime
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
        )
install(DIRECTORY ${vision_spdlog_INC_DIR}/
        DESTINATION include/vision_spdlog
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
        )
install(DIRECTORY ${TYCAMERA_INC_DIR}/
        DESTINATION include/tyCamera
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
        )

# -------------------- 安装库文件 --------------------
# obCamera 库
file(GLOB OBCAMERA_LIB_FILES "${OBCAMERA_LIB_DIR}/libOrbbecSDK.so*")
if(OBCAMERA_LIB_FILES)
  install(FILES ${OBCAMERA_LIB_FILES}
          DESTINATION lib
          PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                      GROUP_READ GROUP_EXECUTE
                      WORLD_READ WORLD_EXECUTE
          )
else()
  message(FATAL_ERROR "[vision_third_party] obCamera库文件不存在: ${OBCAMERA_LIB_DIR}")
endif()

# onnxruntime 库
file(GLOB ONNXRUNTIME_LIB_FILES "${ONNXRUNTIME_LIB_DIR}/libonnxruntime*.so*")
if(ONNXRUNTIME_LIB_FILES)
  install(FILES ${ONNXRUNTIME_LIB_FILES}
          DESTINATION lib
          PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                      GROUP_READ GROUP_EXECUTE
                      WORLD_READ WORLD_EXECUTE
          )
else()
  message(FATAL_ERROR "[vision_third_party] onnxruntime库文件不存在: ${ONNXRUNTIME_LIB_DIR}")
endif()

# vision_spdlog 库
file(GLOB vision_spdlog_LIB_FILES "${vision_spdlog_LIB_DIR}/*.a")
file(GLOB vision_spdlog_SHARED_FILES "${vision_spdlog_LIB_DIR}/*.so*")
if(vision_spdlog_LIB_FILES OR vision_spdlog_SHARED_FILES)
  install(FILES ${vision_spdlog_LIB_FILES} ${vision_spdlog_SHARED_FILES}
          DESTINATION lib
          PERMISSIONS OWNER_READ OWNER_WRITE
                      GROUP_READ
                      WORLD_READ
          )
else()
  message(FATAL_ERROR "[vision_third_party] vision_spdlog库文件不存在: ${vision_spdlog_LIB_DIR}")
endif()

# tyCamera 库
file(GLOB TYCAMERA_LIB_FILES "${TYCAMERA_LIB_DIR}/*.so*")
file(GLOB TYCAMERA_STATIC_FILES "${TYCAMERA_LIB_DIR}/*.a")
if(TYCAMERA_LIB_FILES OR TYCAMERA_STATIC_FILES)
  install(FILES ${TYCAMERA_LIB_FILES} ${TYCAMERA_STATIC_FILES}
          DESTINATION lib
          PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                      GROUP_READ GROUP_EXECUTE
                      WORLD_READ WORLD_EXECUTE
          )
else()
  message(FATAL_ERROR "[vision_third_party] tyCamera库文件不存在: ${TYCAMERA_LIB_DIR}")
endif()

# -------------------- 安装 Config.cmake --------------------
install(FILES ${VISION_THIRD_PARTY_ROOT}/cmake/nlohmannConfig.cmake
        DESTINATION share/nlohmann/cmake/
        )
install(FILES ${VISION_THIRD_PARTY_ROOT}/cmake/obCameraConfig.cmake
        DESTINATION share/obCamera/cmake/
        )
install(FILES ${VISION_THIRD_PARTY_ROOT}/cmake/onnxruntimeConfig.cmake
        DESTINATION share/onnxruntime/cmake/
        )
install(FILES ${VISION_THIRD_PARTY_ROOT}/cmake/vision_spdlogConfig.cmake
        DESTINATION share/vision_spdlog/cmake/
        )
install(FILES ${VISION_THIRD_PARTY_ROOT}/cmake/tyCameraConfig.cmake
        DESTINATION share/tyCamera/cmake/
        )

# -------------------- 导出配置 --------------------
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
