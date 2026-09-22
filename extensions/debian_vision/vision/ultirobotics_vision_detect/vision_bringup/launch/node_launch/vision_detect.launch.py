#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
vision_detect.launch.py - 视觉检测节点启动文件

功能说明：
    启动视觉检测节点，提供目标检测服务。

启动命令：
    ros2 launch vision_bringup vision_detect.launch.py config_file:=/path/to/config.json

启动的节点：
    vision_detect_ros - 视觉检测节点

参数：
    config_file (str) - 配置文件路径（必需）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """启动配置函数（延迟执行）"""
    # 获取 config_file 参数值（必需参数）
    config_file = LaunchConfiguration('config_file')

    # 创建视觉检测节点
    vision_node = Node(
        package="vision_detect_ros",
        executable="vision_detect_ros",
        arguments=[config_file],
        output="full",
        emulate_tty=True,
    )

    return [vision_node]


def generate_launch_description():
    """生成 Launch 描述"""
    return LaunchDescription([
        # 声明 config_file 参数（必需）
        DeclareLaunchArgument(
            'config_file'
        ),
        OpaqueFunction(function=launch_setup),
    ])
