# 1. 定义库版本（按需修改，和实际nlohmann/json版本一致即可）
set(nlohmann_VERSION 1.0.0)
set(nlohmann_VERSION_MAJOR 1)
set(nlohmann_VERSION_MINOR 0)
set(nlohmann_VERSION_PATCH 0)

# 2. 定位头文件目录，并定义兼容变量 nlohmann_INCLUDE_DIR
set(nlohmann_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../include/nlohmann")
# 定义你要的 nlohmann_INCLUDE_DIR 变量（和你的写法匹配）
set(nlohmann_INCLUDE_DIR ${nlohmann_INCLUDE_DIRS})

# 验证头文件存在（关键：确保目录有效）
if(NOT EXISTS "${nlohmann_INCLUDE_DIR}")
    message(FATAL_ERROR "nlohmann头文件目录不存在: ${nlohmann_INCLUDE_DIR}")
endif()

# 3. 创建命名空间目标（核心修改：INTERFACE 适配纯头文件库）
if(NOT TARGET nlohmann::nlohmann)
    add_library(nlohmann::nlohmann INTERFACE IMPORTED) # 仅改这里！
    set_target_properties(nlohmann::nlohmann PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${nlohmann_INCLUDE_DIR}"
        # 若nlohmann需要额外编译宏，可加在这里，使用者会自动继承
        # INTERFACE_COMPILE_DEFINITIONS "JSON_HAS_CPP_17=1"
    )
endif()

# 4. 标记包为已找到（必须，去掉了库相关变量，只校验头文件）
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(nlohmann
    REQUIRED_VARS nlohmann_INCLUDE_DIRS # 纯头文件库，仅校验头文件路径
    VERSION_VAR nlohmann_VERSION
)

# 5. 标记高级变量（修正：只保留存在的nlohmann_INCLUDE_DIR）
mark_as_advanced(nlohmann_INCLUDE_DIR)
