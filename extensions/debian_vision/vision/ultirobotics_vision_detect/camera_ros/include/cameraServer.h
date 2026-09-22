#pragma once
#include <iostream>
#include <string>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include "vision_interface/srv/color_dep_intrinsic.hpp"
#include "camera_drive/camera_rgbd.h"

class cameraServer {
public:
    cameraServer();
    ~cameraServer();
    void imageServer(rclcpp::Node::SharedPtr node, std::string camera_server_name);
    bool init(rclcpp::Node::SharedPtr node,const std::string& config_path);
    void stop();
private:
    using ColorDepIntrinsic = vision_interface::srv::ColorDepIntrinsic;
    using Request = ColorDepIntrinsic::Request;
    using Response = ColorDepIntrinsic::Response;

    void HandleService(
        const std::shared_ptr<Request> request,
        std::shared_ptr<Response> response);

    rclcpp::Service<ColorDepIntrinsic>::SharedPtr vision_server_;
    std::map<std::string, cameraRGBD::Ptr> cameras_;
};
