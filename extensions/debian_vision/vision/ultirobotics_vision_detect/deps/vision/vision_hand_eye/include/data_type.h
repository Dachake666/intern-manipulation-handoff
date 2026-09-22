#pragma once
#ifndef DATA_TYPE_H
#define DATA_TYPE_H
#include<iostream>
#include<opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "cal_logger.h"

class object_pose {
public:
	object_pose() {
		x = 0;
		y = 0;
		z = 0;
		angle_x = 0;
		angle_y = 0;
		angle_z = 0;
		area = 0;
	}
	object_pose(float x1, float y1, float z1, float ax, float ay, float az,float area_tp) {
		x = x1;
		y = y1;
		z = z1;
		angle_x = ax;
		angle_y = ay;
		angle_z = az;
		area = area_tp;
	}
	~object_pose() {
	}
	object_pose& operator=(const object_pose& tp) {
		if (this != &tp) {
			x = tp.x;
			y = tp.y;
			z = tp.z;
			angle_x = tp.angle_x;
			angle_y = tp.angle_y;
			angle_z = tp.angle_z;
			area = tp.area;
		}
		return *this;
	}
	float x;
	float y;
	float z;
	float angle_x;
	float angle_y;
	float angle_z;
    float area;
};

class markPoints {
public:
	markPoints() {
	}
	markPoints(int id, pcl::PointCloud<pcl::PointXYZ>& pts) {
		id_ = id;
		points_ = pts;
	}
	~markPoints() {
	}
	markPoints& operator=(const markPoints& tp) {
		if (this != &tp) {
			id_ = tp.id_;
			points_ = tp.points_;
		}
		return *this;
	}
	pcl::PointCloud<pcl::PointXYZ> points_;
	int id_;
};


class Detection {
public:
    Detection() {
    }
    Detection(cv::Rect box_tp, int class_id_tp, float max_conf, std::vector<float> mask_coeffs_tp) {
        box = box_tp;
        class_id = class_id_tp;
        confidence = max_conf;
        mask_coeffs = mask_coeffs_tp;
    }
    ~Detection() {
    }
    Detection& operator=(const Detection& tp) {
        if (this != &tp) {
            box = tp.box;
            class_id = tp.class_id;
            confidence = tp.confidence;
            mask_coeffs = tp.mask_coeffs;
        }
        return *this;
    }
    cv::Rect box;
    int class_id;
    float confidence;
    std::vector<float> mask_coeffs;  // ÐÂÔö£º±£´æÑÚÄ¤ÏµÊý
};

class yoloseg_detect_result {
public:
    yoloseg_detect_result() {
    }
    ~yoloseg_detect_result() {
    }
    yoloseg_detect_result& operator=(const yoloseg_detect_result& tp) {
        if (this != &tp) {
            box = tp.box;
            class_id = tp.class_id;
            confidence = tp.confidence;
            part_mask = tp.part_mask.clone();
        }
        return *this;
    }
    cv::Rect box;
    int class_id;
    float confidence;
    cv::Mat part_mask;
};

class image_Points {
public:
	image_Points() {
	}
	image_Points(cv::Point2f tmp_2d, cv::Point3f tmp_3d) {
		pt2d = tmp_2d;
		pt3d = tmp_3d;
	}
	~image_Points() {
	}
	image_Points& operator=(const image_Points& tp) {
		if (this != &tp) {
			pt2d = tp.pt2d;
			pt3d = tp.pt3d;
		}
		return *this;
	}
	cv::Point2f pt2d;
	cv::Point3f pt3d;
};

class camera_callback_data{
public:
    camera_callback_data(){
        scale = 0.25;
        error_data = 0;
    }
    ~camera_callback_data(){}
    camera_callback_data& operator=(const camera_callback_data& tp){
        if(this!=&tp){
            color = tp.color.clone();
            dep = tp.dep.clone();
            intrinsic = tp.intrinsic;
            error_data = tp.error_data;
            scale = tp.scale;
        }
        return *this;
    }
    cv::Mat color;
    cv::Mat dep;
    std::vector<float> intrinsic;
    int error_data;
    float scale;
};

struct algorithmParam {
	int box_min_limit = 50;
	int box_max_limit = 150;
	std::string mode_name = "best.onnx";
	bool is_detect_angle = true;
	bool is_detect_material_box = true;
	std::vector<cv::Point3f> material_box_points;
	cv::Point3f material_top_point;
	cv::Point3f material_bottom_point;
	bool is_get_param = false;
    std::vector<std::vector<double>> camera_rt = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    std::vector<double> rt_inv = {1,0,0,0,0,1,0,0,0,0,1,0};
    int hand_eye = 0;
};

extern algorithmParam algorithm_param_global;
extern std::string detect_server_name_global;
void get_rt_inv(std::vector<std::vector<double>>& rt, std::vector<double>& rt_inv);

#endif // !DATA_TYPE_H
