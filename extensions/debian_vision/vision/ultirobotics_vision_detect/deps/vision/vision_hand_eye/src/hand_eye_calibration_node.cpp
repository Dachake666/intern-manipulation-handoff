#include <iostream>
#include "calibration_server.h"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hand_eye_calibration_node");

    // 获取配置文件夹路径
    std::string config_dir = "";
    if (argc > 1) {
        config_dir = argv[1];
        RCLCPP_INFO(node->get_logger(), "Config directory: %s", config_dir.c_str());
    } else {
        RCLCPP_WARN(node->get_logger(), "No config directory provided, using default");
    }


	std::string camera_server_name = "color_dep_intrinsic";
	std::string calibration_server_name = "hand_eye_calibration_server";


    RCLCPP_INFO(node->get_logger(), "Starting Hand-Eye Calibration Node");
    RCLCPP_INFO(node->get_logger(), "Camera Server Name: %s", camera_server_name.c_str());
    RCLCPP_INFO(node->get_logger(), "Calibration Server Name: %s", calibration_server_name.c_str());

    // Create the calibration server instance with config directory
    calibrationServer cal_server(node, camera_server_name, calibration_server_name, config_dir);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
