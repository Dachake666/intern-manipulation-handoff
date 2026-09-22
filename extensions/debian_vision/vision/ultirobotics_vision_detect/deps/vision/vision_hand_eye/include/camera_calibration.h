#pragma once
#ifndef VISITIONDETECT_H
#define VISITIONDETECT_H
#include<iostream>
#include<vector>
#include "markdetector.h"
#include <opencv2/opencv.hpp>
#include "cal_logger.h"
#include "mark_point_position.h"
#include <Eigen/Dense>
#include<string>
#include "data_type.h"
using namespace Eigen;

class pose6d{
public:
    pose6d(){
    }
    ~pose6d(){
    }
    pose6d& operator=(const pose6d& tp){
        if(this!=&tp){
            x = tp.x;
            y = tp.y;
            z = tp.z;
            orientation_x = tp.orientation_x;
            orientation_y = tp.orientation_y;
            orientation_z = tp.orientation_z;
            orientation_w = tp.orientation_w;
        }
        return *this;
    }
    float x;
    float y;
    float z;
    float orientation_x;
    float orientation_y;
    float orientation_z;
    float orientation_w;
};

class pose_euler_angle{
public:
    pose_euler_angle(){
        x = 0;
        y = 0;
        z = 0;
        theta_x = 0;
        theta_y = 0;
        theta_z = 0;
        euler_rot_order_type = 0;
    }
    pose_euler_angle(float px,float py,float pz,float tx,float ty,float tz,float type){
        x = px;
        y = py;
        z = pz;
        theta_x = tx;
        theta_y = ty;
        theta_z = tz;
        euler_rot_order_type = type;
    }
    ~pose_euler_angle(){
    }
    pose_euler_angle& operator=(const pose_euler_angle& tp){
        if(this!=&tp){
            x = tp.x;
            y = tp.y;
            z = tp.z;
            theta_x = tp.theta_x;
            theta_y = tp.theta_y;
            theta_z = tp.theta_z;
            euler_rot_order_type = tp.euler_rot_order_type;
        }
        return *this;
    }
    float x;
    float y;
    float z;
    float theta_x;
    float theta_y;
    float theta_z;
    int euler_rot_order_type;
};

class matchPoint3d {
public:
    matchPoint3d(){}
    ~matchPoint3d() {
        std::vector<cv::Mat>().swap(Tmark2cam);
        std::vector<cv::Mat>().swap(hand_pose);
    }
    matchPoint3d& operator=(const matchPoint3d& tp) {
        if (this != &tp) {
            Tmark2cam = tp.Tmark2cam;
            hand_pose = tp.hand_pose;
        }
        return *this;
    }
    void clear() {
        Tmark2cam.clear();
        hand_pose.clear();
    }
    std::vector<cv::Mat> Tmark2cam;
    std::vector<cv::Mat> hand_pose;
};

cv::Mat pose6dToMat44(const pose6d& pose);

class cameraCalibration{
public:
    cameraCalibration();
    ~cameraCalibration();

    void get_cam_img(pose6d& pose);
    void get_cam_img(camera_callback_data& cam_data,pose6d& hand_pose);
    void find_apriltag(cv::Mat& color,cv::Mat& xyz,pose6d& pose);
    void find_apriltag(cv::Mat& color, cv::Mat& xyz, cv::Mat& pose,std::vector<Marker>& marks);
    void write_image_data(cv::Mat color, cv::Mat dep,std::vector<float> intrinsic, float scale, cv::Mat result, pose6d pose);
    void cam_calibration(std::string file_name,float& total_rot_error,float& mean_trans_error,std::vector<float>& cam_rt);
    void get_mark_positon(std::vector<Marker>& marks, cv::Mat& xyz, std::vector<mark3dPosition>& mark3ds);
    void cancel();
    int get_error_data();
    std::string get_cam_config_path(std::string file_name);
    bool load_camera_config(const std::string& config_abs_path, nlohmann::json& config);
    void cancel_last_detect();
    void setMarkType(const std::string& mark_type);
    void set_workspace_id(const std::string& workspace_id);
    void set_save_path(const std::string& save_path);
    void set_hand_eye_type(int hand_eye_type);
    void set_config_dir(std::string config_dir);

private:
    int error_data_;
    float total_rot_error_;
    float mean_trans_error_;
    std::string save_json_file_;
    std::vector<pose6d> pose_;
    MarkerDetector detector_mark_;
    markPosition mark3d_;

    int cal_index_;
    int save_count_;
    bool app_first_;
    matchPoint3d math_pt3d_;
    std::string save_path_;
    std::string workspace_id_;
    int hand_eye_type_;
};


#endif
