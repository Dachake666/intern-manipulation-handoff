#pragma once
#ifndef SERVER_TOOL_H
#define SERVER_TOOL_H
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include"data_type.h"

class resultSaveIo {
public:
	resultSaveIo(algorithmParam& param);
	~resultSaveIo();
	void thread_write_result_image_data();
	void write_result_image_data(algorithmParam algorithm_param);
	void write_image_data(cv::Mat color, cv::Mat dep, std::vector<float> intrinsic, float scale);
    void write_image_data_xyz(cv::Mat color, cv::Mat dep, std::vector<float> intrinsic, float scale);
	void local_img_detect();
	void get_local_img(camera_callback_data& cam_data);

	std::shared_ptr<Logger> logger_;
private:
	std::string local_img_path_ = "data";
	std::vector<std::string> imgs_path_;
	std::vector<float> local_intrinsic_;
	int local_img_index_;
    float scale_;

	algorithmParam& algorithm_param_global;
};

#endif // !SERVER_TOOL_H
