#pragma once
#ifndef DETECTSERVER_H
#define DETECTSERVER_H
#include<iostream>
#include "rclcpp/rclcpp.hpp"
#include "vision_interface/srv/hand_eye_calibration.hpp"
#include "camera_calibration.h"
#include "cal_logger.h"
#include"cameraClient.h"
#include"local_data_test.h"

class calibrationServer {
public:
    calibrationServer(rclcpp::Node::SharedPtr node,std::string camera_server_name,std::string cal_server_name,std::string config_dir="");
    ~calibrationServer();
    void createServer(rclcpp::Node::SharedPtr node,std::string cal_server_name);
    void stop();
private:
    using HandEyeCalibration = vision_interface::srv::HandEyeCalibration;

    void handle_service(
        const std::shared_ptr<HandEyeCalibration::Request> request,
        std::shared_ptr<HandEyeCalibration::Response> response);
    
    void get_camera_img(camera_callback_data& cam_data, const std::string& camera_id);
    void loadConfig(const std::string& config_dir);

    rclcpp::Service<HandEyeCalibration>::SharedPtr vision_server_;
    cameraClient cam_client_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    int request_type_;
    pose6d hand_pose_;
    std::string save_json_file_;
    std::string save_path_;
    std::string mark_type_;
	int hand_eye_type_;
    cameraCalibration cam_calibration_;
    localDataRead local_data_read_;
};


#endif
