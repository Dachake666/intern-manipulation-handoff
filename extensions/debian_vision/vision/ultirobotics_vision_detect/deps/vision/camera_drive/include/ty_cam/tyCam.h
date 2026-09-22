#pragma once
#ifndef TYCAM_H
#define TYCAM_H
#define OPENCV_DEPENDENCIES
#include<iostream>
#include<vector>
#include "common/common.hpp"
#include "opencv2/opencv.hpp"
#include "TYApi.h"
//#include"vision_log.h"
#include <vision_utils/vision_log.h>
#include "../camera_interface.h"
//#include "common/MatViewer.hpp"
//#include "TYImageProc.h"

struct CamInfo
{
	//	char* frameBuffer[3];
	std::vector<char> frameBuffer[2];
	TY_INTERFACE_HANDLE hIface;
	TY_DEV_HANDLE hDevice;
	//	uint32_t frameSize;
	TY_CAMERA_CALIB_INFO depth_calib;
	TY_CAMERA_CALIB_INFO color_calib;
	//std::vector<TY_VECT_3F> p3d;
	cv::Mat color;
	TY_FRAME_DATA frame;
	std::string IP;
	float           scale_unit;
	bool            isTof;
};

struct tyParam {
	std::string ip_;
	int color_exposure_;
	int color_resolution_width_;
	int color_resolution_height_;
	int color_analog_gain_;
	int dep_resolution_width_;
	int dep_resolution_height_;
	int left_ir_exposure_;
	int left_gain_;
	int left_analog_gain_;
	int right_ir_exposure_;
	int right_gain_;
	int right_analog_gain_;
	int power_level_;
};

class TyCamera :public camInterface
{

public:
	TyCamera();
	void Init_camera();
	~TyCamera();
	bool config_camera(nlohmann::json& config) override;
	bool open() override;
	bool close() override;
	void get_cam_data(cv::Mat& color_img, cv::Mat& dep_img, std::vector<float>& intrinsic) override;
	bool get_img(CamInfo& cams, cv::Mat& img, cv::Mat& dep, std::vector<float>& intrinsic);
	bool set_camera_param(TY_DEV_HANDLE device);
    int get_cam_state() override;
    float get_cam_scale() override;
    //void set_log(std::shared_ptr<Logger> logger) override;
private:
	CamInfo cam_;
	tyParam cam_param_;
	std::atomic<int> connet_state_{ -1 };
	bool is_offline_ = false ;
};





#endif // !TYCAM_H
