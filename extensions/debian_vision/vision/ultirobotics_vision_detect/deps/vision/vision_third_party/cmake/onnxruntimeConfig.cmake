# 1. 定义库版本（按需修改）
set(onnxruntime_VERSION 1.23.1)
set(onnxruntime_VERSION_MAJOR 1)
set(onnxruntime_VERSION_MINOR 0)
set(onnxruntime_VERSION_PATCH 0)

# 2. 定位头文件目录，并定义兼容变量 onnxruntime_INCLUDE_DIR
set(onnxruntime_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../include/onnxruntime")
# 定义你要的 onnxruntime_INCLUDE_DIR 变量（和你的写法匹配）
set(onnxruntime_INCLUDE_DIR ${onnxruntime_INCLUDE_DIRS})

# 验证头文件存在
if(NOT EXISTS "${onnxruntime_INCLUDE_DIR}")
    message(FATAL_ERROR "onnxruntime头文件目录不存在: ${onnxruntime_INCLUDE_DIR}")
endif()

# 3. 定位.so库文件，并定义兼容变量 onnxruntime_LIB
set(onnxruntime_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/../../../lib/libonnxruntime.so")
# 定义你要的 onnxruntime_LIB 变量（和你的写法匹配）
set(onnxruntime_LIB ${onnxruntime_LIBRARY})

# 验证库文件存在
if(NOT EXISTS "${onnxruntime_LIB}")
    message(FATAL_ERROR "onnxruntime共享库文件不存在: ${onnxruntime_LIB}")
endif()

# 4. （可选但推荐）创建命名空间目标（保留现代写法能力）
if(NOT TARGET onnxruntime::onnxruntime)
    add_library(onnxruntime::onnxruntime SHARED IMPORTED)
    set_target_properties(onnxruntime::onnxruntime PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_INCLUDE_DIR}"
        IMPORTED_LOCATION "${onnxruntime_LIB}"
    )
endif()

# 5. 标记包为已找到（必须）
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(onnxruntime
    REQUIRED_VARS onnxruntime_INCLUDE_DIRS onnxruntime_LIBRARY
    VERSION_VAR onnxruntime_VERSION
)

# 6. 可选：将变量导出为全局变量（确保使用者能直接访问）
mark_as_advanced(onnxruntime_INCLUDE_DIR onnxruntime_LIB)
