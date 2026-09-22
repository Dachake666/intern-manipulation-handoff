#pragma once
#ifndef IMAGE_TOOL
#define IMAGE_TOOL
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include"data_type.h"

class detectorMaskProcessor {
public:
	detectorMaskProcessor(algorithmParam& param);
	~detectorMaskProcessor();
	bool choose_correct_object_seg(std::vector<yoloseg_detect_result>& detect_box, cv::Mat& xyz, std::vector<int>& out_index);
	bool choose_correct_object_seg_hight(std::vector<yoloseg_detect_result>& detect_box, cv::Mat& xyz, std::vector<int>& out_index);
	void select_items_in_material_box(yoloseg_detect_result& material_seg, std::vector<yoloseg_detect_result>& object_seg);
	bool get_object_downsampling_point(cv::Mat& xyz_img, yoloseg_detect_result& object, pcl::PointCloud<pcl::PointXYZ>& object_point);
	void calculate_coverage_area(std::vector<yoloseg_detect_result>& detect_result, std::vector<object_pose>& pose,cv::Mat& xyz);
	void remove_object_from_crop_region(std::vector<yoloseg_detect_result>& detect_box);
	std::shared_ptr<Logger> logger_;
private:
	algorithmParam& algorithm_param_global;
};

#endif // !IMAGE_TOOL
