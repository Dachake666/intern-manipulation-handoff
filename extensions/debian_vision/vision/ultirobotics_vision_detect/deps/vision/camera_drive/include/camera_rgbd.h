#pragma once
#ifndef CAMERA_RGBD_H
#define CAMERA_RGBD_H
#include"camera_interface.h"
#include"ob_cam/obCamera.h"
#include"ty_cam/tyCam.h"

class cameraRGBD{
public:
    using Ptr = std::shared_ptr<cameraRGBD>;
    cameraRGBD();
    ~cameraRGBD();
    bool init(const std::string& config_path,std::string log_path);
    bool open();
    bool close();
    void get_cam_data(cv::Mat& color_img,
        cv::Mat& dep_img,
        std::vector<float>& intrinsic);
    int get_cam_state();
    float get_cam_scale();
    void set_log(std::string path);

private:
    camInterface* camera_ = nullptr;
};

#endif // CAMERA_RGBD_H
