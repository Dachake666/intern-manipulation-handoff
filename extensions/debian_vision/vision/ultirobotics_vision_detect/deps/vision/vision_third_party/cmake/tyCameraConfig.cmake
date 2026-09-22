# 1. 定义库版本（按需修改）
set(tyCamera_VERSION 4.0.0)
set(tyCamera_VERSION_MAJOR 1)
set(tyCamera_VERSION_MINOR 0)
set(tyCamera_VERSION_PATCH 0)

# 2. 定位头文件目录，并定义兼容变量 tyCamera_INCLUDE_DIR
set(tyCamera_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../include/tyCamera")
# 定义你要的 tyCamera_INCLUDE_DIR 变量（和你的写法匹配）
set(tyCamera_INCLUDE_DIR ${tyCamera_INCLUDE_DIRS})

# 验证头文件存在
if(NOT EXISTS "${tyCamera_INCLUDE_DIR}")
    message(FATAL_ERROR "tyCamera头文件目录不存在: ${tyCamera_INCLUDE_DIR}")
endif()

# 3. 定位.so库文件，并定义兼容变量 tyCamera_LIB
set(tyCamera_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/../../../lib/libtycam.so")
# 定义你要的 tyCamera_LIB 变量（和你的写法匹配）
set(tyCamera_LIB ${tyCamera_LIBRARY})

# 验证库文件存在
if(NOT EXISTS "${tyCamera_LIB}")
    message(FATAL_ERROR "tyCamera共享库文件不存在: ${tyCamera_LIB}")
endif()

# 4. （可选但推荐）创建命名空间目标（保留现代写法能力）
if(NOT TARGET tyCamera::tyCamera)
    add_library(tyCamera::tyCamera SHARED IMPORTED)
    set_target_properties(tyCamera::tyCamera PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${tyCamera_INCLUDE_DIR}"
        IMPORTED_LOCATION "${tyCamera_LIB}"
    )
endif()

# 5. 标记包为已找到（必须）
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(tyCamera
    REQUIRED_VARS tyCamera_INCLUDE_DIRS tyCamera_LIBRARY
    VERSION_VAR tyCamera_VERSION
)

# 6. 可选：将变量导出为全局变量（确保使用者能直接访问）
mark_as_advanced(tyCamera_INCLUDE_DIR tyCamera_LIB)
