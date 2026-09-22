#pragma once
#ifndef SERVER_TOOL_H
#define SERVER_TOOL_H
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include "data_type.h"

class localDataRead {
public:
    localDataRead();
    ~localDataRead();
    void local_img_detect();
    void get_local_img(camera_callback_data& cam_data,std::vector<float>& hand_pose);
private:
    int local_img_index_;
    std::string local_img_path_ ;
    std::vector<std::string> imgs_path_;
    std::vector<float> local_intrinsic_;
    std::vector<std::string> hand_pose_path_;
    float scale_;
};

#endif // !SERVER_TOOL_H
