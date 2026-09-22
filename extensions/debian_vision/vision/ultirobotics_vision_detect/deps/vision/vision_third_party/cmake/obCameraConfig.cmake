# 1. 定义库版本（按需修改）
set(obCamera_VERSION 2.4.11)
set(obCamera_VERSION_MAJOR 1)
set(obCamera_VERSION_MINOR 0)
set(obCamera_VERSION_PATCH 0)

# 2. 定位头文件目录，并定义兼容变量 obCamera_INCLUDE_DIR
set(obCamera_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../include/obCamera")
# 定义你要的 obCamera_INCLUDE_DIR 变量（和你的写法匹配）
set(obCamera_INCLUDE_DIR ${obCamera_INCLUDE_DIRS})

# 验证头文件存在
if(NOT EXISTS "${obCamera_INCLUDE_DIR}")
    message(FATAL_ERROR "obCamera头文件目录不存在: ${obCamera_INCLUDE_DIR}")
endif()

# 3. 定位.so库文件，并定义兼容变量 obCamera_LIB
set(obCamera_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/../../../lib/libOrbbecSDK.so")
# 定义你要的 obCamera_LIB 变量（和你的写法匹配）
set(obCamera_LIB ${obCamera_LIBRARY})

# 验证库文件存在
if(NOT EXISTS "${obCamera_LIB}")
    message(FATAL_ERROR "obCamera共享库文件不存在: ${obCamera_LIB}")
endif()

# 4. （可选但推荐）创建命名空间目标（保留现代写法能力）
if(NOT TARGET obCamera::obCamera)
    add_library(obCamera::obCamera SHARED IMPORTED)
    set_target_properties(obCamera::obCamera PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${obCamera_INCLUDE_DIR}"
        IMPORTED_LOCATION "${obCamera_LIB}"
    )
endif()

# 5. 标记包为已找到（必须）
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(obCamera
    REQUIRED_VARS obCamera_INCLUDE_DIRS obCamera_LIBRARY
    VERSION_VAR obCamera_VERSION
)

# 6. 可选：将变量导出为全局变量（确保使用者能直接访问）
mark_as_advanced(obCamera_INCLUDE_DIR obCamera_LIB)
