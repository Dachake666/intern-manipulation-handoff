#pragma once
#ifndef IMAGE_UTIL_H
#define IMAGE_UTIL_H
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include"data_type.h"
class ImageProcess {
public:
	ImageProcess(algorithmParam& param);
	~ImageProcess();
	bool get_xyz_from_intrinsic(cv::Mat& dep, std::vector<float>& intrinsic, float scale, cv::Mat& xyz);
	void draw_seg_result(std::vector<yoloseg_detect_result>& result, cv::Mat& img, cv::Scalar color);
	void draw_objec_rect(cv::Mat& img, yoloseg_detect_result& object);
	void draw_box_limit_points(cv::Mat& img, std::vector<float>& intrinsic);
	void draw_multiple_pose_result(cv::Mat& img, std::vector<object_pose>& pose, std::vector<float>& intrinsic, cv::Scalar rgb);
	int find_correct_material_box(std::vector<yoloseg_detect_result>& box_result, cv::Mat& img);
	bool set_background_image(cv::Mat& img);

	void draw_rio_dotted_line(cv::Mat& img, std::vector<cv::Point2f>& pts, cv::Scalar line_color);
	std::shared_ptr<Logger> logger_;
private:
	algorithmParam& algorithm_param_global;
};

#endif // !IMAGE_UTIL_H
