#pragma once
#ifndef CAMERA_CLIENT_MANAGER_H
#define CAMERA_CLIENT_MANAGER_H

#include <iostream>
#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include "vision_interface/srv/color_dep_intrinsic.hpp"

class cameraClientManager {
public:
    cameraClientManager();
    ~cameraClientManager();
    
    void init(rclcpp::Node::SharedPtr node, const std::string& service_name);
    
    bool get_camera_image(
        const std::string& camera_id,
        cv::Mat& color,
        cv::Mat& dep,
        std::vector<float>& intrinsic,
        float& scale,
        int& error_data,
        int timeout_ms = 10000);
    
    void release_camera(const std::string& camera_id);

private:
    using ColorDepIntrinsic = vision_interface::srv::ColorDepIntrinsic;
    
    struct CameraClient {
        rclcpp::Client<ColorDepIntrinsic>::SharedPtr client;
        std::mutex mutex;
        std::condition_variable cv;
        std::atomic<bool> in_use{false};
        bool request_pending{false};
        bool response_ready{false};
        
        cv::Mat color;
        cv::Mat dep;
        std::vector<float> intrinsic;
        float scale;
        int error_data;
        bool success;
    };
    
    void response_callback(
        const std::string& camera_id,
        rclcpp::Client<ColorDepIntrinsic>::SharedFuture future);
    
    rclcpp::Node::SharedPtr node_;
    std::string service_name_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    std::map<std::string, std::shared_ptr<CameraClient>> camera_clients_;
    std::mutex manager_mutex_;
};

#endif // CAMERA_CLIENT_MANAGER_H
