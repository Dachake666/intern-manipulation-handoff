#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
app.launch.py - 主应用启动文件

功能说明：
    同时启动相机驱动和视觉检测两个节点。

启动命令：
    ros2 launch vision_bringup app.launch.py config_file:=/path/to/config.json

启动的节点：
    1. camera_ros - 相机驱动节点
    2. vision_detect_ros - 视觉检测节点

参数：
    config_file (str) - 配置文件路径（必需，两个节点使用相同的配置文件）
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    """启动配置函数（延迟执行）"""
    # 获取 config_file 参数值（必需参数）
    config_file = LaunchConfiguration('config_file')

    # 获取 vision_bringup 包的安装路径
    bringup_dir = get_package_share_directory("vision_bringup")

    # 构建节点启动文件目录路径
    launch_dir = os.path.join(bringup_dir, "launch/node_launch")

    # 构建启动动作列表
    action_list = [
        # 启动相机驱动节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "camera.launch.py")),
            launch_arguments={
                "config_file": config_file,
            }.items(),
        ),
        # 启动视觉检测节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "vision_detect.launch.py")),
            launch_arguments={
                "config_file": config_file,
            }.items(),
        ),
    ]

    return action_list


def generate_launch_description():
    """生成 Launch 描述"""
    return LaunchDescription([
        # 声明 config_file 参数（必需）
        DeclareLaunchArgument(
            'config_file'
        ),
        OpaqueFunction(function=launch_setup),
    ])
