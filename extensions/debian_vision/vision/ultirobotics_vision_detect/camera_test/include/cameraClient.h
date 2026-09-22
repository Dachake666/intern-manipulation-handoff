#pragma once
#ifndef CAMERACLIENT_H
#define CAMERACLIENT_H

#include <iostream>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include "vision_interface/srv/color_dep_intrinsic.hpp"

class cameraClient {
public:
    cameraClient();
    ~cameraClient();
    
    void imageClient(rclcpp::Node::SharedPtr node, std::string camera_server_name);
    void send_request(const std::string& camera_id);
    bool get_camera_img(cv::Mat& color, cv::Mat& dep, std::vector<float>& intrinsic, float& scale, int& error_data);

private:
    using ColorDepIntrinsic = vision_interface::srv::ColorDepIntrinsic;
    
    void response_callback(rclcpp::Client<ColorDepIntrinsic>::SharedFuture future);
    
    rclcpp::Client<ColorDepIntrinsic>::SharedPtr client_ptr_;
    
    cv::Mat color_;
    cv::Mat dep_;
    std::vector<float> intrinsic_;
    
    std::mutex m_mutex_;
    bool got_img_;
    std::atomic<bool> is_getting_{false};
    std::atomic<bool> is_get_img_success_{false};
    std::condition_variable m_cv_;
    int error_data_;
    float scale_;
};

#endif // CAMERACLIENT_H
