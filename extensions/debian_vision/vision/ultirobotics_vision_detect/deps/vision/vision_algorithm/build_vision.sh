#!/bin/bash
set -e

# 从环境变量获取路径（由 CMake 传递）
SCRIPT_DIR="${SOURCE_DIR}"
WORKSPACE_DIR="${WORKSPACE_DIR}"
REAL_BUILD_DIR="${REAL_BUILD_DIR}"
PACKAGE_NAME="vision_algorithm"

NEED_REBUILD=0

if [ ! -f "$REAL_BUILD_DIR/libvision_algorithm.so" ]; then
    NEED_REBUILD=1
    echo "[vision_algorithm] .so not found"
else
    SO_TIME=$(stat -c %Y "$REAL_BUILD_DIR/libvision_algorithm.so" 2>/dev/null || echo 0)
    
    CURRENT_SRC_COUNT=$(find "$SCRIPT_DIR/src" \( -name "*.cpp" -o -name "*.c" \) 2>/dev/null | wc -l)
    CACHED_SRC_COUNT=$(cat "$REAL_BUILD_DIR/.src_count" 2>/dev/null || echo 0)
    if [ "$CURRENT_SRC_COUNT" != "$CACHED_SRC_COUNT" ]; then
        NEED_REBUILD=1
        echo "[vision_algorithm] Source count: $CACHED_SRC_COUNT -> $CURRENT_SRC_COUNT"
    fi
    
    if [ "$NEED_REBUILD" -eq 0 ]; then
        for src_file in $(find "$SCRIPT_DIR/src" \( -name "*.cpp" -o -name "*.c" \) 2>/dev/null); do
            SRC_TIME=$(stat -c %Y "$src_file" 2>/dev/null || echo 0)
            if [ "$SRC_TIME" -gt "$SO_TIME" ]; then
                NEED_REBUILD=1
                break
            fi
        done
    fi
    
    if [ "$NEED_REBUILD" -eq 0 ]; then
        CURRENT_HDR_COUNT=$(find "$SCRIPT_DIR/include" \( -name "*.h" -o -name "*.hpp" \) 2>/dev/null | wc -l)
        CACHED_HDR_COUNT=$(cat "$REAL_BUILD_DIR/.hdr_count" 2>/dev/null || echo 0)
        if [ "$CURRENT_HDR_COUNT" != "$CACHED_HDR_COUNT" ]; then
            NEED_REBUILD=1
            echo "[vision_algorithm] Header count: $CACHED_HDR_COUNT -> $CURRENT_HDR_COUNT"
        fi
    fi
    
    if [ "$NEED_REBUILD" -eq 0 ]; then
        for header_file in $(find "$SCRIPT_DIR/include" \( -name "*.h" -o -name "*.hpp" \) 2>/dev/null); do
            HEADER_TIME=$(stat -c %Y "$header_file" 2>/dev/null || echo 0)
            if [ "$HEADER_TIME" -gt "$SO_TIME" ]; then
                NEED_REBUILD=1
                break
            fi
        done
    fi
    
    if [ "$NEED_REBUILD" -eq 0 ] && [ -f "$SCRIPT_DIR/CMakeLists.txt.build" ]; then
        CMAKE_TIME=$(stat -c %Y "$SCRIPT_DIR/CMakeLists.txt.build" 2>/dev/null || echo 0)
        if [ "$CMAKE_TIME" -gt "$SO_TIME" ]; then
            NEED_REBUILD=1
        fi
    fi
fi

if [ "$NEED_REBUILD" -eq 1 ]; then
    echo "[vision_algorithm] Building..."
    echo "[vision_algorithm] REAL_BUILD_DIR: $REAL_BUILD_DIR"
    
    mkdir -p "$REAL_BUILD_DIR"
    rm -rf "$REAL_BUILD_DIR"/*
    
    source /opt/ros/humble/setup.bash
    if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        source "$WORKSPACE_DIR/install/setup.bash"
    fi
    
    cp "$SCRIPT_DIR/CMakeLists.txt.build" "$REAL_BUILD_DIR/CMakeLists.txt"
    
    cd "$REAL_BUILD_DIR"
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DSOURCE_DIR="$SCRIPT_DIR" \
          -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}:$WORKSPACE_DIR/install" .
    make
    
    find "$SCRIPT_DIR/src" \( -name "*.cpp" -o -name "*.c" \) 2>/dev/null | wc -l > "$REAL_BUILD_DIR/.src_count"
    find "$SCRIPT_DIR/include" \( -name "*.h" -o -name "*.hpp" \) 2>/dev/null | wc -l > "$REAL_BUILD_DIR/.hdr_count"
else
    echo "[vision_algorithm] Skipping rebuild"
fi
