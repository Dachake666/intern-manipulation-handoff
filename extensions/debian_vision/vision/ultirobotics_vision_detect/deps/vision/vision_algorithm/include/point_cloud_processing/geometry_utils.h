#pragma once
#ifndef GEOMETRY_UTLS_H
#define GEOMETRY_UTLS_H
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

float pointToLineDistance(const cv::Point2f& p0, const cv::Point2f& p1, const cv::Point2f& p2);
Eigen::Quaterniond xyzEulerDegToQuaternion(double angleX_deg, double angleY_deg, double angleZ_deg);
cv::Mat poseVectorToTransformMatrix_q(const std::vector<float>& pose);
bool is_point_in_polygon(const cv::Point2i& pt, const std::vector<cv::Point2f>& vertices);
#endif // !GEOMETRY_UTLS_H
