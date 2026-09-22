#include <rclcpp/rclcpp.hpp>
#include "cameraServer.h"

int main(int ac, char** av)
{
  rclcpp::init(ac, av);
  rclcpp::executors::MultiThreadedExecutor executor;
  //auto node = std::make_shared<vision::ros::CameraNode>();
  auto node = rclcpp::Node::make_shared("camera_node");
  
  if (ac < 2)
  {
    RCLCPP_ERROR(node->get_logger(), "please input config path");
    return -1;
  }
  cameraServer cam_server;

  if (!cam_server.init(node,av[1]))
  {
    RCLCPP_ERROR(node->get_logger(), "node init failed, program exit!");
    return -1;
  }
  
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
