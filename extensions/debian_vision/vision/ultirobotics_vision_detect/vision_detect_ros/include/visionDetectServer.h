#pragma once
#ifndef VISION_DETECT_SERVER_H
#define VISION_DETECT_SERVER_H

#include <iostream>
#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>
#include <vector>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "vision_interface/action/vision_detect_result.hpp"
#include "vision_object_detect.h"
#include "data_type.h"
#include "vision_utils/io.h"
#include "cameraClientManager.h"

struct request_data{
    int request_detect;
    std::vector<std::string> text_prompt;
    std::string workspace_id;
    std::vector<float> object_size;
    std::vector<float> hand_poses;
    void clear(){
        request_detect = 0;
        text_prompt.clear();
        workspace_id = "";
        object_size.clear();
        hand_poses.clear();
    }
    /*
    request_data& operator=(const request_data& other) {
        if (this == &other)
            return *this;
        request_detect = other.request_detect;
        text_prompt = other.text_prompt;
        workspace_id = other.workspace_id;
    }
    */
};

struct object_Info{
    float position_x;
    float position_y;
    float position_z;
    float orientation_x;
    float orientation_y;
    float orientation_z;
    float orientation_w;
    float area;
    float length;
    float width;
    float height;
    int overlap_count;
    std::vector<float> overlap_ratio;
    std::vector<int> overlap_index;
    std::vector<cv::Point3f> outside_polygon_vertex;
    int index;
    void clear(){
        overlap_count = 0;
        overlap_ratio.clear();
        overlap_index.clear();
        outside_polygon_vertex.clear();
    }
};
 
struct response_data{
    bool success;
    int count;
    std::vector<object_Info> object;
    int material_pt_size;
    std::vector<cv::Point3f> material_points;
    int error_data;
    int hand_and_eye = 0;
    std::vector<float> hand_pose;
    std::vector<float> box_crop_pose;
    std::vector<float> box_crop_size;
    void clear(){
        success = false;
        count = 0;
        object.clear();
        material_pt_size = 0;
        material_points.clear();
        error_data = -1;
        hand_pose.clear();
        box_crop_pose.clear();
        box_crop_size.clear();
    }
};


class visionDetectServer {
public:
    visionDetectServer();
    ~visionDetectServer();
    
    bool init(rclcpp::Node::SharedPtr node, const std::string& config_path);
    void startServer(rclcpp::Node::SharedPtr node, const std::string& service_name);
    void stop();

private:
    using VisionDetectResult = vision_interface::action::VisionDetectResult;
    using Goal = VisionDetectResult::Goal;
    using Result = VisionDetectResult::Result;
    using Feedback = VisionDetectResult::Feedback;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const VisionDetectResult::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<VisionDetectResult>> goal_handle);
    void executeCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<VisionDetectResult>> goal_handle);
    
    void get_rt_inv(std::vector<std::vector<double>>& rt, std::vector<double>& rt_inv);
    std::string get_timestamp();
    std::string get_date();
    void set_response_data(
        std::shared_ptr<Result> result,
        std::vector<object_pose>& poses,
        int error_data,
        algorithmParam& param);
    //void set_response_zero(std::shared_ptr<Response> response);
    
    void save_json(std::string response_file, response_data& data);
    void save_request_json(std::string response_file, request_data& data);
    void save_request_data(std::string path, std::string time_date, std::string time_ms, request_data data);
    void save_response_json(std::string path, std::string time_date, std::string time_ms, response_data data);
    
    rclcpp_action::Server<VisionDetectResult>::SharedPtr detect_server_;
    
    std::map<std::string, std::string> workspace_camera_map_;
    std::map<std::string, algorithmParam> workspace_param_map_;
    std::map<std::string, std::shared_ptr<visionObjectDetect>> workspace_detector_map_;
    
    std::shared_ptr<cameraClientManager> camera_manager_;
    
    rclcpp::Node::SharedPtr node_;
    std::mutex mutex_;
    std::string save_path_;
    request_data save_request_data_;
    response_data save_response_data_;
};

#endif // VISION_DETECT_SERVER_H
