# 1. 定义库版本（按需修改，和实际vision_spdlog版本一致即可）
set(vision_spdlog_VERSION 1.9.2)
set(vision_spdlog_VERSION_MAJOR 1)
set(vision_spdlog_VERSION_MINOR 0)
set(vision_spdlog_VERSION_PATCH 0)

# 2. 定位头文件目录，并定义兼容变量 vision_spdlog_INCLUDE_DIR
set(vision_spdlog_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../include/vision_spdlog")
set(vision_spdlog_INCLUDE_DIR ${vision_spdlog_INCLUDE_DIRS}) # 适配你的写法

# 验证头文件存在
if(NOT EXISTS "${vision_spdlog_INCLUDE_DIR}")
    message(FATAL_ERROR "vision_spdlog头文件目录不存在: ${vision_spdlog_INCLUDE_DIR}")
endif()

# 3. 定位.a静态库文件，并定义兼容变量 vision_spdlog_LIB
set(vision_spdlog_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/../../../lib/libspdlog.a")
set(vision_spdlog_LIB ${vision_spdlog_LIBRARY})

if(NOT EXISTS "${vision_spdlog_LIB}")
    message(FATAL_ERROR "vision_spdlog静态库文件不存在: ${vision_spdlog_LIB}")
endif()

set(fmt_INCLUDE_DIR "${vision_spdlog_INCLUDE_DIR}/fmt")
set(fmt_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/../../../lib/libfmt.so")

# 验证本地fmt文件存在
if(NOT EXISTS "${fmt_INCLUDE_DIR}")
    message(FATAL_ERROR "本地fmt头文件目录不存在: ${fmt_INCLUDE_DIR}")
endif()
if(NOT EXISTS "${fmt_LIBRARY}")
    message(FATAL_ERROR "本地fmt库文件不存在: ${fmt_LIBRARY}")
endif()

# ========== 核心修改2：手动创建fmt::fmt目标（使用本地库，不依赖系统） ==========
if(NOT TARGET fmt::fmt)
    add_library(fmt::fmt SHARED IMPORTED GLOBAL)
    set_target_properties(fmt::fmt PROPERTIES
        # fmt头文件路径（本地）
        INTERFACE_INCLUDE_DIRECTORIES "${fmt_INCLUDE_DIR}"
        # fmt库文件路径（你拷贝的libfmt.so）
        IMPORTED_LOCATION "${fmt_LIBRARY}"
        # 忽略SONAME检查（因为是拷贝改名的文件）
        IMPORTED_NO_SONAME TRUE
    )
endif()

# 4. 创建命名空间目标（核心修改：STATIC 适配.a静态库）
if(NOT TARGET vision_spdlog::vision_spdlog)
    add_library(vision_spdlog::vision_spdlog STATIC IMPORTED) # 仅改这里！
    set_target_properties(vision_spdlog::vision_spdlog PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${vision_spdlog_INCLUDE_DIR}"
        IMPORTED_LOCATION "${vision_spdlog_LIB}"
        # ========== 核心修改3：强制关联本地fmt库 ==========
        INTERFACE_LINK_LIBRARIES "fmt::fmt"
        # 兜底：直接添加本地fmt库的链接选项，确保-lfmt生效
        INTERFACE_LINK_OPTIONS "-L${CMAKE_CURRENT_LIST_DIR}/../../../lib -lfmt"
    )
endif()

# 注释掉原有的系统fmt查找逻辑（不再需要）
# # 4.1 查找ROS2系统中的fmt库
# if(NOT TARGET fmt::fmt)
#     find_package(fmt REQUIRED)
# endif()

# # 4.2 将fmt依赖添加到vision_spdlog目标中（调用时自动继承）
# target_link_libraries(vision_spdlog::vision_spdlog PUBLIC fmt::fmt)

# 5. CMake标准：标记包为已找到
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(vision_spdlog
    REQUIRED_VARS vision_spdlog_INCLUDE_DIRS vision_spdlog_LIBRARY
    VERSION_VAR vision_spdlog_VERSION
)

# 6. 标记高级变量（不影响使用，仅优化CMake界面）
mark_as_advanced(vision_spdlog_INCLUDE_DIR vision_spdlog_LIB)
