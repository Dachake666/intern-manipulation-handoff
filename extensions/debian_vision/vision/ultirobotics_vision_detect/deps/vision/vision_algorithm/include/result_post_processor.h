#pragma once
#ifndef RESULT_POST_PROCESSOR_H
#define RESULT_POST_PROCESSOR_H
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include"data_type.h"

class resultPostProcessor {
public:
	resultPostProcessor();
	~resultPostProcessor();
	void remove_results_outside_material_box(std::vector<object_pose>& multiple_obj_pose,algorithmParam& algorithm_param_global);
	bool is_outside_material_box(object_pose& multiple_obj_pose,algorithmParam& algorithm_param_global);
};

#endif