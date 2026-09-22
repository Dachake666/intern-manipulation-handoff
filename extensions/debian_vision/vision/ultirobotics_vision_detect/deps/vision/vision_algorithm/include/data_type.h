#pragma once
#ifndef DATA_TYPE_H
#define DATA_TYPE_H
#include<iostream>
#include<opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "vision_utils/vision_log.h"
#define invalid_dep_z 1000000

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
        index = 0;
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
            overlap_area_ratio = tp.overlap_area_ratio;
			overlap_index = tp.overlap_index;
			index = tp.index;
            length = tp.length;
            width = tp.width;
            height = tp.height;
            polygon_vertex = tp.polygon_vertex;
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
    std::vector<float> overlap_area_ratio;
	std::vector<int> overlap_index;
    int length = 0;
    int width = 0;
    int height = 0;
	int index;
    std::vector<cv::Point3f> polygon_vertex;
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
    std::vector<float> mask_coeffs;  
};

class yoloseg_detect_result {
public:
    yoloseg_detect_result() {
        class_id = 0;
        confidence = 0;
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

struct material_box_param {
    bool is_detect = 1;
    std::vector<cv::Point3f> material_box_points;
    cv::Point2f img_position = cv::Point2f(0.5,0.5);
    int box_length = 600;
    int box_width = 400;
    bool is_using_box_size = 0;
};
struct background_image {
    std::string path;
    bool is_using = 0;
};
struct detect_rio_2d {
    bool is_using = 0;
    //float height = 0;
    std::vector<cv::Point2f> pts;
    float inside_ratio = 0.5;
};

struct algorithmParam {
	std::string model_path = "best.onnx";
	bool is_detect_angle = true;
    std::vector<std::vector<double>> camera_rt = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    std::vector<double> rt_inv = {1,0,0,0,0,1,0,0,0,0,1,0};
    int hand_and_eye = 0;
    std::vector<std::vector<double>> camera_rt_eye = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    std::vector<float> hand_pose = {0,0,0,0,0,0,1};
	camera_callback_data cam_data;
    std::vector<object_pose> multiple_object_pose;
    cv::Mat result_img;
    std::string workstation;
    std::string result_data_path;
    int result_error_data;
    std::string local_time_ms;
    std::string local_time_date;
    std::string save_image_type = "png";
    bool save_color = true;
    bool save_depth = true;
    bool save_pred = true;
    std::string sam3_server_ip = "127.0.0.1:5555";
    bool is_using_box_crop = 0;
    std::vector<float> box_crop_pose;
    cv::Point3f box_crop_size = cv::Point3f(0,0,0);
    int box_Z_Range = 100;
    int box_PointLimit = 3000;
    material_box_param material_box;
    background_image back_ground_img;
    detect_rio_2d seg_rio;
    int result_candidate_number = 10;

    algorithmParam& operator=(const algorithmParam& other) {
        if (this == &other) return *this;
        model_path = other.model_path;
        is_detect_angle = other.is_detect_angle;
        camera_rt = other.camera_rt;
        rt_inv = other.rt_inv;
        hand_and_eye = other.hand_and_eye;
        cam_data = other.cam_data;
        multiple_object_pose = other.multiple_object_pose;
        result_img = other.result_img.clone();
        workstation = other.workstation;
        result_data_path = other.result_data_path;
        result_error_data = other.result_error_data;
        local_time_ms = other.local_time_ms;
        local_time_date = other.local_time_date;
        save_image_type = other.save_image_type;
        save_color = other.save_color;
        save_depth = other.save_depth;
        save_pred = other.save_pred;
        sam3_server_ip = other.sam3_server_ip;
        camera_rt_eye = other.camera_rt_eye;
        hand_pose = other.hand_pose;
		is_using_box_crop = other.is_using_box_crop;
        box_crop_pose = other.box_crop_pose;
        box_crop_size = other.box_crop_size;
        box_Z_Range = other.box_Z_Range;
        box_PointLimit = other.box_PointLimit;
        material_box = other.material_box;
        back_ground_img = other.back_ground_img;
        seg_rio = other.seg_rio;
        result_candidate_number = other.result_candidate_number;
        return *this;
    }
};

#endif // !DATA_TYPE_H
