#pragma once
#ifndef YOLOV8_SEG_H
#define YOLOV8_SEG_H
#include <iostream>
#include<vector>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include "../data_type.h"

class yolov8seg {
public:
    yolov8seg();
	void init(std::string model_path);
	~yolov8seg();

    void onnx_detect(cv::Mat& img,std::vector<yoloseg_detect_result>& reult, std::vector<yoloseg_detect_result>& box_reult);
    void get_input_tensor_values_data(cv::Mat& img, std::vector<float>& input_tensor_values);
    std::vector<Detection> postprocess(const float* output, int ori_w, int ori_h);
    cv::Mat generate_mask(const std::vector<float>& mask_coeffs, cv::Mat& proto_mat, int proto_w, int proto_h,
        const cv::Rect& box, int ori_w, int ori_h);


    Ort::Env env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "YOLOv8");
    Ort::SessionOptions session_options_;

    Ort::MemoryInfo memory_info_ = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    Ort::Session* session_ = NULL;


    //std::vector<std::string> class_name_ = {"box","clothes","package"};
    //std::vector<std::string> class_name_ = { "box","clothes" };
    std::vector<std::string> class_name_ = { "box"};

private:
    const int INPUT_WIDTH = 640;
    const int INPUT_HEIGHT = 640;
    const float CONF_THRESH = 0.55;
    float nms_threshold_;
    float scale_;
    int top_;
    int left_;
    int num_classes_;
};

#endif