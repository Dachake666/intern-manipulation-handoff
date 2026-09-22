#include <rclcpp/rclcpp.hpp>
#include "visionDetectServer.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("vision_detect_server");
    
    if (argc < 2) {
        RCLCPP_ERROR(node->get_logger(), "Please input config path");
        return -1;
    }
    
    visionDetectServer detect_server;
    
    if (!detect_server.init(node, argv[1])) {
        RCLCPP_ERROR(node->get_logger(), "Vision detect server init failed, program exit!");
        return -1;
    }
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
