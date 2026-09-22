#pragma once
#ifndef OBCAMERA_H
#include<atomic>
#include <libobsensor/ObSensor.hpp>
#include "libobsensor/hpp/Utils.hpp"
#include<iostream>
#include "opencv2/opencv.hpp"
#include "../camera_interface.h"
using namespace std;

class obCamera:public camInterface {
public:
	obCamera();
	~obCamera();
	void cam_reboot();
	void cam_connect();
	void get_cam_img(cv::Mat& color_img,cv::Mat& dep_img,std::vector<float>& intrinsic);
	bool config_camera(nlohmann::json& config) override;
	bool open() override;
	bool close() override;
	void get_cam_data(cv::Mat& color_img, cv::Mat& dep_img, std::vector<float>& intrinsic) override;
    int get_cam_state() override;
    float get_cam_scale() override;
    //void set_log(std::shared_ptr<Logger> logger) override;
private:
	std::shared_ptr<ob::Device> dev_;
	std::shared_ptr<ob::Pipeline> pipe_;
	bool is_soft_trigger_;
	int camera_height_;
	int camera_width_;
    std::atomic<int> is_connect_{-1};
	std::vector<float> intrinsic_;
};

#endif // !OBCAMERA_H
