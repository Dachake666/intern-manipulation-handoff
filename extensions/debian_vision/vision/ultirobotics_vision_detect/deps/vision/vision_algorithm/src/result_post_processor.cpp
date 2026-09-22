#include "result_post_processor.h"
#include "geometry_utils.h"
resultPostProcessor::resultPostProcessor() {

}
resultPostProcessor::~resultPostProcessor() {

}

bool fitPlane(const std::vector<cv::Point3f>& points, cv::Vec4f& plane)
{
    if (points.size() < 3) return false;

    cv::Mat A(points.size(), 3, CV_32F);
    cv::Mat b(points.size(), 1, CV_32F);

    for (int i = 0; i < points.size(); ++i) {
        A.at<float>(i, 0) = points[i].x;
        A.at<float>(i, 1) = points[i].y;
        A.at<float>(i, 2) = 1.0f;
        b.at<float>(i, 0) = -points[i].z;
    }

    cv::Mat x;
    if (!cv::solve(A, b, x, cv::DECOMP_SVD)) return false;

    plane[0] = x.at<float>(0); // a
    plane[1] = x.at<float>(1); // b
    plane[2] = 1.0f;           // c (������ z ����)
    plane[3] = x.at<float>(2); // d
    return true;
}

// ���ܣ����� p ��ƽ�淨����ͶӰ������ target_pt ��ƽ����ԭƽ�桿����ƽ��
cv::Point3f projectToParallelPlane(
    const cv::Point3f& p,
    const cv::Vec4f& plane,
    const cv::Point3f& target_pt)
{
    // ԭƽ�棺a*x + b*y + c*z + d = 0
    float a = plane[0];
    float b = plane[1];
    float c = plane[2];
    float d = plane[3];

    // ƽ��ƽ�湫ʽ��a*x + b*y + c*z = a*xt + b*yt + c*zt
    float plane_val_target = a * target_pt.x + b * target_pt.y + c * target_pt.z;
    float plane_val_p = a * p.x + b * p.y + c * p.z + d;

    // ��������ĸ
    float denom = a * a + b * b + c * c;
    if (denom < 1e-6) return p;

    // ͶӰϵ�� t
    float t = (plane_val_target - plane_val_p) / denom;

    // �ط������ƶ��õ�ͶӰ���3D��
    cv::Point3f p_proj;
    p_proj.x = p.x + a * t;
    p_proj.y = p.y + b * t;
    p_proj.z = p.z + c * t;

    return p_proj;
}

void projectBoxToParallelPlane(
    const std::vector<cv::Point3f>& box_points,
    cv::Vec4f plane,
    const cv::Point3f& target_pt,
    std::vector<cv::Point2f>& projected_2d_points)
{
    projected_2d_points.clear();

    // 2. ÿ�����ط�����ͶӰ��ƽ��ƽ��
    for (const auto& p : box_points) {
        cv::Point3f p3d = projectToParallelPlane(p, plane, target_pt);
        projected_2d_points.emplace_back(p3d.x, p3d.y); // ȡx,y���2D��
    }
    std::sort(projected_2d_points.begin(), projected_2d_points.end(),
        [](const cv::Point2f& a, const cv::Point2f& b) {
            return a.x < b.x;
        });
    if (projected_2d_points[0].y < projected_2d_points[1].y) {
        cv::Point2f tmp = projected_2d_points[0];
        projected_2d_points[0] = projected_2d_points[1];
        projected_2d_points[1] = tmp;
    }
    if (projected_2d_points[2].y > projected_2d_points[3].y) {
        cv::Point2f tmp = projected_2d_points[2];
        projected_2d_points[2] = projected_2d_points[3];
        projected_2d_points[3] = tmp;
    }
}


void resultPostProcessor::remove_results_outside_material_box(std::vector<object_pose>& multiple_obj_pose,algorithmParam& algorithm_param_global) {
    if (multiple_obj_pose.size() < 1)
        return;
    std::vector<cv::Point3f>& box_points = algorithm_param_global.material_box.material_box_points;
    if (box_points.size() != 4)
        return;
    cv::Vec4f plane;
    fitPlane(box_points, plane);
    std::vector<object_pose> temp_pose;
    cv::Point3f target_pt;
    std::vector<cv::Point2f> box_2d_pts;
    cv::Point2f temp_center;
    bool is_out = 0;
    for (int i = 0; i < multiple_obj_pose.size(); ++i) {
        is_out = 0;
        target_pt.x = multiple_obj_pose[i].x;
        target_pt.y = multiple_obj_pose[i].y;
        target_pt.z = multiple_obj_pose[i].z;
        temp_center.x = multiple_obj_pose[i].x;
        temp_center.y = multiple_obj_pose[i].y;
        projectBoxToParallelPlane(box_points, plane, target_pt, box_2d_pts);
        for (int j = 0; j < box_2d_pts.size(); ++j) {
            float dist = pointToLineDistance(temp_center, box_2d_pts[j], box_2d_pts[(j + 1) % box_2d_pts.size()]);
            if (dist > -10) {
                is_out = 1;
                break;
            }
        }
        if (0 == is_out) {
            temp_pose.push_back(multiple_obj_pose[i]);
        }
    }
    multiple_obj_pose.swap(temp_pose);
}

bool resultPostProcessor::is_outside_material_box(object_pose& obj_pose,algorithmParam& algorithm_param_global) {
    if (!algorithm_param_global.material_box.is_detect)
        return false;
    std::vector<cv::Point3f>& box_points = algorithm_param_global.material_box.material_box_points;
    if (box_points.size() != 4)
        return true;
    cv::Vec4f plane;
    fitPlane(box_points, plane);
    std::vector<object_pose> temp_pose;
    cv::Point3f target_pt;
    std::vector<cv::Point2f> box_2d_pts;
    cv::Point2f temp_center;
    bool is_out = 0;
    target_pt.x = obj_pose.x;
    target_pt.y = obj_pose.y;
    target_pt.z = obj_pose.z;
    temp_center.x = obj_pose.x;
    temp_center.y = obj_pose.y;
    projectBoxToParallelPlane(box_points, plane, target_pt, box_2d_pts);
    for (int j = 0; j < box_2d_pts.size(); ++j) {
        float dist = pointToLineDistance(temp_center, box_2d_pts[j], box_2d_pts[(j + 1) % box_2d_pts.size()]);
        if (dist > -10) {
            is_out = 1;
            break;
        }
    }
    if (1 == is_out) {
        return true;
    }
    return false;

}