#include"image_processing/image_process.h"
ImageProcess::ImageProcess(algorithmParam& param):algorithm_param_global(param){
}
ImageProcess::~ImageProcess() {
}

bool ImageProcess::get_xyz_from_intrinsic(cv::Mat& dep, std::vector<float>& intrinsic, float scale, cv::Mat& xyz) {
    xyz = cv::Mat(dep.size(), CV_32FC3);//CV_16UC3  
    ushort* depdata = (ushort*)dep.data;
    float* xyzdata = (float*)xyz.data;
    float dep_z;
    float dep_x;
    float dep_y;
    cv::Point3f world_pt(0, 0, 0);
    cv::Point3f limit_min(0,0,0);
    cv::Point3f limit_max(0, 0, 0);
    float problematic_limit = 0;
    bool using_crop = false;
    int count_limit_pts = 0;
    float rotate_x = 0;
    float rotate_y = 0;
    float dx, dy;
    float cos_z = 1;
    float sin_z = 0;
    if (algorithm_param_global.is_using_box_crop && algorithm_param_global.box_crop_pose.size() == 6) {
        using_crop = true;
        limit_min.x = algorithm_param_global.box_crop_pose[0] - algorithm_param_global.box_crop_size.x / 2.0;
        limit_min.y = algorithm_param_global.box_crop_pose[1] - algorithm_param_global.box_crop_size.y / 2.0;
        limit_min.z = algorithm_param_global.box_crop_pose[2] - algorithm_param_global.box_crop_size.z / 2.0;

        limit_max.x = algorithm_param_global.box_crop_pose[0] + algorithm_param_global.box_crop_size.x / 2.0;
        limit_max.y = algorithm_param_global.box_crop_pose[1] + algorithm_param_global.box_crop_size.y / 2.0;
        limit_max.z = algorithm_param_global.box_crop_pose[2] + algorithm_param_global.box_crop_size.z / 2.0;
        problematic_limit = limit_max.z - algorithm_param_global.box_Z_Range;

        float rad_angle = algorithm_param_global.box_crop_pose[5] * M_PI / 180.0;
        cos_z = cos(rad_angle);
        sin_z = sin(rad_angle);
    }
    for (int y = 0; y < dep.rows; ++y)
        for (int x = 0; x < dep.cols; ++x) {
            dep_z = depdata[y * dep.cols + x] * scale;
            if (0 == dep_z) {
                xyzdata[3 * y * xyz.cols + 3 * x] = invalid_dep_z;
                xyzdata[3 * y * xyz.cols + 3 * x + 1] = invalid_dep_z;
                xyzdata[3 * y * xyz.cols + 3 * x + 2] = invalid_dep_z;
            }
            else {
                dep_x = (x - intrinsic[2]) * dep_z / intrinsic[0];
                dep_y = (y - intrinsic[5]) * dep_z / intrinsic[4];
                world_pt.x = algorithm_param_global.camera_rt[0][0] * dep_x + algorithm_param_global.camera_rt[0][1] * dep_y + algorithm_param_global.camera_rt[0][2] * dep_z + algorithm_param_global.camera_rt[0][3];
                world_pt.y = algorithm_param_global.camera_rt[1][0] * dep_x + algorithm_param_global.camera_rt[1][1] * dep_y + algorithm_param_global.camera_rt[1][2] * dep_z + algorithm_param_global.camera_rt[1][3];
                world_pt.z = algorithm_param_global.camera_rt[2][0] * dep_x + algorithm_param_global.camera_rt[2][1] * dep_y + algorithm_param_global.camera_rt[2][2] * dep_z + algorithm_param_global.camera_rt[2][3];
                if (using_crop) {
                    dx = world_pt.x - algorithm_param_global.box_crop_pose[0];
                    dy = world_pt.y - algorithm_param_global.box_crop_pose[1];
                    rotate_x = dx * cos_z + dy * sin_z;
                    rotate_y = -dx * sin_z + dy * cos_z;
                    rotate_x = rotate_x + algorithm_param_global.box_crop_pose[0];
                    rotate_y = rotate_y + algorithm_param_global.box_crop_pose[1];

                    if (rotate_x > limit_min.x && rotate_x < limit_max.x && rotate_y> limit_min.y && rotate_y < limit_max.y &&
                        world_pt.z > limit_min.z && world_pt.z < limit_max.z) {
                        xyzdata[3 * y * xyz.cols + 3 * x] = world_pt.x;
                        xyzdata[3 * y * xyz.cols + 3 * x + 1] = world_pt.y;
                        xyzdata[3 * y * xyz.cols + 3 * x + 2] = world_pt.z;
                        if (world_pt.z > problematic_limit) {
                            count_limit_pts++;
                        }
                    }
                    else {
                        xyzdata[3 * y * xyz.cols + 3 * x] = invalid_dep_z;
                        xyzdata[3 * y * xyz.cols + 3 * x + 1] = invalid_dep_z;
                        xyzdata[3 * y * xyz.cols + 3 * x + 2] = invalid_dep_z;
                    }
                }
                else {
                    xyzdata[3 * y * xyz.cols + 3 * x] = world_pt.x;
                    xyzdata[3 * y * xyz.cols + 3 * x + 1] = world_pt.y;
                    xyzdata[3 * y * xyz.cols + 3 * x + 2] = world_pt.z;
                }
            }
        }
    if (using_crop && count_limit_pts > algorithm_param_global.box_PointLimit) {
        LOG_ERROR("count_limit_pts  > box_PointLimit  {}   {}", count_limit_pts, algorithm_param_global.box_PointLimit);
        return false;
    }
    return true;
}

void ImageProcess::draw_seg_result(std::vector<yoloseg_detect_result>& result, cv::Mat& img, cv::Scalar color) {
    for (int i = 0; i < result.size(); ++i) {
        cv::Mat mark = result[i].part_mask;
        int color_b = rand() % 256;
        int color_g = rand() % 256;
        int color_r = rand() % 256;

        uchar* img_data = img.data;//img.data;
        uchar* m_data = mark.data;
        int x_start = result[i].box.x;
        int y_start = result[i].box.y;
        int img_col = img.cols;
        int img_row = img.rows;
        int img_x = 0;
        int img_y = 0;
        for (int y = 0; y < mark.rows; ++y)
            for (int x = 0; x < mark.cols; ++x) {
                if (m_data[y * mark.cols + x] > 0) {
                    img_x = x + x_start;
                    img_y = y + y_start;
                    if (img_x >= 0 && img_x < img_col && img_y >= 0 && img_y < img_row) {
                        img_data[(img_y)*img.cols * 3 + 3 * (img_x)] = 0.5 * img_data[(img_y)*img.cols * 3 + 3 * (img_x)] + 0.5 * color_b;
                        img_data[(img_y)*img.cols * 3 + 3 * (img_x)+1] = 0.5 * img_data[(img_y)*img.cols * 3 + 3 * (img_x)+1] + 0.5 * color_g;
                        img_data[(img_y)*img.cols * 3 + 3 * (img_x)+2] = 0.5 * img_data[(img_y)*img.cols * 3 + 3 * (img_x)+2] + 0.5 * color_r;
                    }
                }
            }

        cv::rectangle(img, result[i].box, color, 2);
    }
}

void ImageProcess::draw_multiple_pose_result(cv::Mat& img, std::vector<object_pose>& pose, std::vector<float>& intrinsic, cv::Scalar rgb) {
    for (int i = 0; i < pose.size(); ++i){
        LOG_WARN("result_pose  !!! {}  {}  {}  {}  {}  {}  {}",i, pose[i].x, pose[i].y, pose[i].z, pose[i].angle_x, pose[i].angle_y, pose[i].angle_z);
        float pt_x = algorithm_param_global.rt_inv[0] * pose[i].x + algorithm_param_global.rt_inv[1] * pose[i].y + algorithm_param_global.rt_inv[2] * pose[i].z + algorithm_param_global.rt_inv[3];
        float pt_y = algorithm_param_global.rt_inv[4] * pose[i].x + algorithm_param_global.rt_inv[5] * pose[i].y + algorithm_param_global.rt_inv[6] * pose[i].z + algorithm_param_global.rt_inv[7];
        float pt_z = algorithm_param_global.rt_inv[8] * pose[i].x + algorithm_param_global.rt_inv[9] * pose[i].y + algorithm_param_global.rt_inv[10] * pose[i].z + algorithm_param_global.rt_inv[11];
        cv::Point2f result_pixel;
        result_pixel.x = intrinsic[0] * pt_x / pt_z + intrinsic[2];
        result_pixel.y = intrinsic[4] * pt_y / pt_z + intrinsic[5];
        LOG_WARN("center_point_in_camera_coordinate  !!! {}  {}  {}  {}  {}  {}", pt_x, pt_y, pt_z, result_pixel.x, result_pixel.y, pose[i].area);
        ////////////////////////////////////////////
        int k;
        for (int j = 0; j < pose[i].polygon_vertex.size(); ++j) {
            k = (j + 1) % (pose[i].polygon_vertex.size());

            float pt_x = algorithm_param_global.rt_inv[0] * pose[i].polygon_vertex[j].x + algorithm_param_global.rt_inv[1] * pose[i].polygon_vertex[j].y + algorithm_param_global.rt_inv[2] * pose[i].polygon_vertex[j].z + algorithm_param_global.rt_inv[3];
            float pt_y = algorithm_param_global.rt_inv[4] * pose[i].polygon_vertex[j].x + algorithm_param_global.rt_inv[5] * pose[i].polygon_vertex[j].y + algorithm_param_global.rt_inv[6] * pose[i].polygon_vertex[j].z + algorithm_param_global.rt_inv[7];
            float pt_z = algorithm_param_global.rt_inv[8] * pose[i].polygon_vertex[j].x + algorithm_param_global.rt_inv[9] * pose[i].polygon_vertex[j].y + algorithm_param_global.rt_inv[10] * pose[i].polygon_vertex[j].z + algorithm_param_global.rt_inv[11];
            cv::Point2f result_pixel;
            result_pixel.x = intrinsic[0] * pt_x / pt_z + intrinsic[2];
            result_pixel.y = intrinsic[4] * pt_y / pt_z + intrinsic[5];

            float pt_x2 = algorithm_param_global.rt_inv[0] * pose[i].polygon_vertex[k].x + algorithm_param_global.rt_inv[1] * pose[i].polygon_vertex[k].y + algorithm_param_global.rt_inv[2] * pose[i].polygon_vertex[k].z + algorithm_param_global.rt_inv[3];
            float pt_y2 = algorithm_param_global.rt_inv[4] * pose[i].polygon_vertex[k].x + algorithm_param_global.rt_inv[5] * pose[i].polygon_vertex[k].y + algorithm_param_global.rt_inv[6] * pose[i].polygon_vertex[k].z + algorithm_param_global.rt_inv[7];
            float pt_z2 = algorithm_param_global.rt_inv[8] * pose[i].polygon_vertex[k].x + algorithm_param_global.rt_inv[9] * pose[i].polygon_vertex[k].y + algorithm_param_global.rt_inv[10] * pose[i].polygon_vertex[k].z + algorithm_param_global.rt_inv[11];
            cv::Point2f result_pixel2;
            result_pixel2.x = intrinsic[0] * pt_x2 / pt_z2 + intrinsic[2];
            result_pixel2.y = intrinsic[4] * pt_y2 / pt_z2 + intrinsic[5];
            cv::line(img, result_pixel, result_pixel2, cv::Scalar(0, 0, 255), 2);
        }

        cv::circle(img, result_pixel, 10, rgb, 3);
        cv::putText(img, std::to_string(i), result_pixel, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }
}

int ImageProcess::find_correct_material_box(std::vector<yoloseg_detect_result>& box_result, cv::Mat& img) {
    float img_mid_x = img.cols * algorithm_param_global.material_box.img_position.x;
    float img_mid_y = img.rows * algorithm_param_global.material_box.img_position.y;
    int index = 0;

    float box_mid_x;
    float box_mid_y;
    float dist_xy = 0;
    float min_xy = 1000000000;
    for (int i = 0; i < box_result.size(); ++i) {
        box_mid_x = box_result[i].box.x + box_result[i].box.width / 2;
        box_mid_y = box_result[i].box.y + box_result[i].box.height / 2;
        dist_xy = fabs(box_mid_x - img_mid_x) + fabs(box_mid_y - img_mid_y);
        if (dist_xy < min_xy) {
            min_xy = dist_xy;
            index = i;
        }
    }
    yoloseg_detect_result tmp_box = box_result[index];
    box_result.clear();
    box_result.push_back(tmp_box);
    return 0;
}

void ImageProcess::draw_box_limit_points(cv::Mat& img, std::vector<float>& intrinsic) {
    cv::Point2f result_pixel;
    float x;
    float y;
    float z;
    for (int i = 0; i < algorithm_param_global.material_box.material_box_points.size(); ++i) {
        x = algorithm_param_global.rt_inv[0] * algorithm_param_global.material_box.material_box_points[i].x + algorithm_param_global.rt_inv[1] * algorithm_param_global.material_box.material_box_points[i].y + algorithm_param_global.rt_inv[2] * algorithm_param_global.material_box.material_box_points[i].z + algorithm_param_global.rt_inv[3];
        y = algorithm_param_global.rt_inv[4] * algorithm_param_global.material_box.material_box_points[i].x + algorithm_param_global.rt_inv[5] * algorithm_param_global.material_box.material_box_points[i].y + algorithm_param_global.rt_inv[6] * algorithm_param_global.material_box.material_box_points[i].z + algorithm_param_global.rt_inv[7];
        z = algorithm_param_global.rt_inv[8] * algorithm_param_global.material_box.material_box_points[i].x + algorithm_param_global.rt_inv[9] * algorithm_param_global.material_box.material_box_points[i].y + algorithm_param_global.rt_inv[10] * algorithm_param_global.material_box.material_box_points[i].z + algorithm_param_global.rt_inv[11];
        result_pixel.x = intrinsic[0] * x / z + intrinsic[2];
        result_pixel.y = intrinsic[4] * y / z + intrinsic[5];
        cv::circle(img, result_pixel, 10, cv::Scalar(255, 0, 0), 2);
    }
}

void ImageProcess::draw_objec_rect(cv::Mat& img, yoloseg_detect_result& object) {
    cv::rectangle(img, object.box, cv::Scalar(0, 0, 255), 3);
}

bool ImageProcess::set_background_image(cv::Mat& img) {
    if (!algorithm_param_global.back_ground_img.is_using)
        return true;
    cv::Mat bk_img = cv::imread(algorithm_param_global.back_ground_img.path);
    std::cout<<algorithm_param_global.back_ground_img.path<<std::endl;
    //std::cout<<bk_img.size()<<std::endl;
    if (bk_img.empty() ||algorithm_param_global.box_crop_pose.size()<6) {        
        std::cout<<"bk_img is empty or box_crop_pose is empty"<<std::endl;
        return false;
    }
    cv::resize(bk_img, bk_img, img.size());
    std::vector<cv::Point3f> crop_box_pt;
    cv::Point3f limit_min(0, 0, 0);
    cv::Point3f limit_max(0, 0, 0);
    limit_min.x = algorithm_param_global.box_crop_pose[0] - algorithm_param_global.box_crop_size.x / 2.0;
    limit_min.y = algorithm_param_global.box_crop_pose[1] - algorithm_param_global.box_crop_size.y / 2.0;
    limit_min.z = algorithm_param_global.box_crop_pose[2] - algorithm_param_global.box_crop_size.z / 2.0;

    limit_max.x = algorithm_param_global.box_crop_pose[0] + algorithm_param_global.box_crop_size.x / 2.0;
    limit_max.y = algorithm_param_global.box_crop_pose[1] + algorithm_param_global.box_crop_size.y / 2.0;
    limit_max.z = algorithm_param_global.box_crop_pose[2] + algorithm_param_global.box_crop_size.z / 2.0;

    crop_box_pt.emplace_back(limit_min.x, limit_min.y, limit_max.z);
    crop_box_pt.emplace_back(limit_max.x, limit_min.y, limit_max.z);
    crop_box_pt.emplace_back(limit_max.x, limit_max.y, limit_max.z);
    crop_box_pt.emplace_back(limit_min.x, limit_max.y, limit_max.z);
    cv::Point2f center((limit_min.x + limit_max.x) / 2, (limit_min.y + limit_max.y) / 2);
    float cos_z = cos(algorithm_param_global.box_crop_pose[5] * M_PI / 180.0);
    float sin_z = sin(algorithm_param_global.box_crop_pose[5] * M_PI / 180.0);
    for (int i = 0; i < crop_box_pt.size(); ++i) {
        float px = crop_box_pt[i].x - center.x;
        float py = crop_box_pt[i].y - center.y;

        float rx = px * cos_z - py * sin_z;
        float ry = px * sin_z + py * cos_z;
        crop_box_pt[i].x = rx + center.x;
        crop_box_pt[i].y = ry + center.y;
    }
    std::vector<cv::Point2f> crop_box_2d_pt;
    cv::Point2f img_pt;
    for (int i = 0; i < crop_box_pt.size(); ++i) {
        float pt_x = algorithm_param_global.rt_inv[0] * crop_box_pt[i].x + algorithm_param_global.rt_inv[1] * crop_box_pt[i].y + algorithm_param_global.rt_inv[2] * crop_box_pt[i].z + algorithm_param_global.rt_inv[3];
        float pt_y = algorithm_param_global.rt_inv[4] * crop_box_pt[i].x + algorithm_param_global.rt_inv[5] * crop_box_pt[i].y + algorithm_param_global.rt_inv[6] * crop_box_pt[i].z + algorithm_param_global.rt_inv[7];
        float pt_z = algorithm_param_global.rt_inv[8] * crop_box_pt[i].x + algorithm_param_global.rt_inv[9] * crop_box_pt[i].y + algorithm_param_global.rt_inv[10] * crop_box_pt[i].z + algorithm_param_global.rt_inv[11];
        img_pt.x = algorithm_param_global.cam_data.intrinsic[0] * pt_x / pt_z + algorithm_param_global.cam_data.intrinsic[2];
        img_pt.y = algorithm_param_global.cam_data.intrinsic[4] * pt_y / pt_z + algorithm_param_global.cam_data.intrinsic[5];
        crop_box_2d_pt.push_back(img_pt);
    }
    cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
    std::vector<cv::Point> poly;
    for (auto& p : crop_box_2d_pt) {
        poly.emplace_back(cvRound(p.x), cvRound(p.y));
    }
    fillPoly(mask, poly, cv::Scalar(255));
    bk_img.copyTo(img, ~mask);
    return true;
}

void drawAdaptiveDottedSegment(cv::Mat& img, cv::Point2f p1, cv::Point2f p2, cv::Scalar color, int thickness)
{
    cv::Point2f dir = p2 - p1;
    float total_len = cv::norm(dir);
    if (total_len < 1.0f)
        return;

    cv::Point2f unit = dir / total_len;

    // 根据线段总长动态分配虚线单元：总单元数控制在 6 ~ 20 之间
    int unit_count = total_len / 15.0f;
    if (unit_count < 6)
        unit_count = 6;
    if (unit_count > 20)
        unit_count = 20;
    float single_unit_len = total_len / unit_count;

    // 一个虚线单元 = 实线段 + 空白段，实线占单元60%，空白占40%
    float dash_len = single_unit_len * 0.6f;
    float space_len = single_unit_len * 0.4f;

    float cur_dist = 0.0f;
    while (cur_dist < total_len)
    {
        cv::Point2f seg_start = p1 + unit * cur_dist;
        float seg_end_dist = std::min(cur_dist + dash_len, total_len);
        cv::Point2f seg_end = p1 + unit * seg_end_dist;

        cv::line(img, seg_start, seg_end, color, thickness);
        cur_dist += dash_len + space_len;
    }
}


void ImageProcess::draw_rio_dotted_line(cv::Mat& img, std::vector<cv::Point2f>& pts, cv::Scalar line_color) {
    if (img.empty() || pts.size() < 2)
        return;
    if (!algorithm_param_global.seg_rio.is_using)
        return;
    const int line_thick = 2;

    size_t pt_num = pts.size();
    cv::Point2f pt1, pt2;
    for (size_t i = 0; i < pt_num; ++i)
    {
        size_t next_idx = (i + 1) % pt_num;
        pt1.x = pts[i].x * img.cols;
        pt1.y = pts[i].y * img.rows;
        pt2.x = pts[next_idx].x * img.cols;
        pt2.y = pts[next_idx].y * img.rows;
        drawAdaptiveDottedSegment(img, pt1, pt2, line_color, line_thick);
    }
}