#pragma once
#ifndef CAMERA_INTERFACE_H
#define CAMERA_INTERFACE_H
#include<atomic>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include "vision_utils/vision_log.h"
class camInterface {
public:
    //using Ptr = std::shared_ptr<camInterface>;
    camInterface() = default;
    virtual ~camInterface() = default;
    camInterface(const camInterface&) = delete;
    camInterface& operator=(const camInterface&) = delete;

    virtual bool config_camera(nlohmann::json& config) = 0;
    virtual bool open() = 0;
    virtual bool close() = 0;
    virtual void get_cam_data(cv::Mat& color_img,
        cv::Mat& dep_img,
        std::vector<float>& intrinsic) = 0;
    virtual int get_cam_state() = 0;
    virtual float get_cam_scale() = 0;
    void set_log(std::string path){
        logger_ = std::make_shared<Logger>(path);
    }
    std::shared_ptr<Logger> logger_;
};


#endif // !CAMERA_INTERFACE_H
