#pragma once
#ifndef CAMERASERVER_H
#define CAMERASERVER_H
#include<iostream>
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include "vision_interface/srv/color_dep_intrinsic.hpp"

class cameraClient {
public:
    cameraClient();
    ~cameraClient();
    
    void init(rclcpp::Node::SharedPtr node, const std::string& service_name);
    
    bool get_camera_image(
        const std::string& camera_id,
        cv::Mat& color,
        cv::Mat& dep,
        std::vector<float>& intrinsic,
        float& scale,
        int& error_data,
        int timeout_ms = 10000);
    
    void release();

private:
    using ColorDepIntrinsic = vision_interface::srv::ColorDepIntrinsic;
    
    void response_callback(
        rclcpp::Client<ColorDepIntrinsic>::SharedFuture future);
    
    rclcpp::Node::SharedPtr node_;
    std::string service_name_;
    rclcpp::Client<ColorDepIntrinsic>::SharedPtr client_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    
    std::mutex mutex_;
    std::condition_variable cv_;
    
    bool request_pending_{false};
    bool response_ready_{false};
    
    cv::Mat color_;
    cv::Mat dep_;
    std::vector<float> intrinsic_;
    float scale_;
    int error_data_;
    bool success_;
};

#endif // !CAMERASERVER_H