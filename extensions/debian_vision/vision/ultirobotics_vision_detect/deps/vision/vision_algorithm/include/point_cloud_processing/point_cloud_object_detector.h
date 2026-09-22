#pragma once
#ifndef POINT_CLOUD_OBJECT_DETECTOR_H
#define POINT_CLOUD_OBJECT_DETECTOR_H
#define NOMINMAX
#undef max
#undef min
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h> 
#include <pcl/point_cloud.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include"data_type.h"

class ObjectPointDetect {
public:
	ObjectPointDetect(algorithmParam& param);
	~ObjectPointDetect();
	bool get_material_box_4point(yoloseg_detect_result& box, cv::Mat& xyz, std::vector<float>& intrinsic, std::vector<cv::Point3f>& box_points);
	bool is_object_above_material_box(yoloseg_detect_result& box, cv::Mat& xyz);
	void point_cloud_stat_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>& cloudOut, int MeanK, float STDthre);
	bool calculate_object_pose(yoloseg_detect_result& object_seg, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<cv::Point3d>& center_pt, std::vector < cv::Point3f>& angle_plane, std::vector<int>& points_size, std::vector<cv::Point3i>& plane_size, std::vector<cv::Point3f>& polygon_vertex);
	std::shared_ptr<Logger> logger_;
private:
	algorithmParam& algorithm_param_global;
};


#endif // !POINT_CLOUD_OBJECT_DETECTOR_H
