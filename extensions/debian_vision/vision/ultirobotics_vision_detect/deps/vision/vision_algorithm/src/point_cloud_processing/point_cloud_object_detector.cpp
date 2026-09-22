#include <pcl/segmentation/region_growing.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include "point_cloud_processing/point_cloud_object_detector.h"
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>

bool fit_plane_by_least_squares(std::vector<double>& xjParameters, pcl::PointCloud<pcl::PointXYZ>::Ptr xjData) {
    try {
        xjParameters.clear();
        int count = xjData->size();
        if (count < 3)
            return false;

        double meanX = 0, meanY = 0, meanZ = 0;
        double meanXX = 0, meanYY = 0, meanZZ = 0;
        double meanXY = 0, meanXZ = 0, meanYZ = 0;
        for (int i = 0; i < count; i++) {
            meanX += xjData->points[i].x;
            meanY += xjData->points[i].y;
            meanZ += xjData->points[i].z;
            meanXX += xjData->points[i].x * xjData->points[i].x;
            meanYY += xjData->points[i].y * xjData->points[i].y;
            meanZZ += xjData->points[i].z * xjData->points[i].z;
            meanXY += xjData->points[i].x * xjData->points[i].y;
            meanXZ += xjData->points[i].x * xjData->points[i].z;
            meanYZ += xjData->points[i].y * xjData->points[i].z;
        }
        meanX /= count, meanY /= count, meanZ /= count;
        meanXX /= count, meanYY /= count, meanZZ /= count;
        meanXY /= count, meanXZ /= count, meanYZ /= count;
        /* eigenvector */
        Eigen::Matrix3d eMat;
        eMat(0, 0) = meanXX - meanX * meanX; eMat(0, 1) = meanXY - meanX * meanY; eMat(0, 2) = meanXZ - meanX * meanZ;
        eMat(1, 0) = meanXY - meanX * meanY; eMat(1, 1) = meanYY - meanY * meanY; eMat(1, 2) = meanYZ - meanY * meanZ;
        eMat(2, 0) = meanXZ - meanX * meanZ; eMat(2, 1) = meanYZ - meanY * meanZ; eMat(2, 2) = meanZZ - meanZ * meanZ;
        Eigen::EigenSolver<Eigen::Matrix3d> xjMat(eMat);
        Eigen::Matrix3d eValue = xjMat.pseudoEigenvalueMatrix();
        Eigen::Matrix3d eVector = xjMat.pseudoEigenvectors();
        /* the eigenvector corresponding to the minimum eigenvalue */
        double v1 = eValue(0, 0);
        double v2 = eValue(1, 1);
        double v3 = eValue(2, 2);
        int minNumber = 0;
        if ((abs(v2) <= abs(v1)) && (abs(v2) <= abs(v3)))
            minNumber = 1;

        if ((abs(v3) <= abs(v1)) && (abs(v3) <= abs(v2)))
            minNumber = 2;
        double A = eVector(0, minNumber);
        double B = eVector(1, minNumber);
        double C = eVector(2, minNumber);
        double D = -(A * meanX + B * meanY + C * meanZ);
        /* result */
        if (C < 0)
            A *= -1.0, B *= -1.0, C *= -1.0, D *= -1.0;

        xjParameters.push_back(A);
        xjParameters.push_back(B);
        xjParameters.push_back(C);
        xjParameters.push_back(D);
        xjParameters.push_back(meanX);
        xjParameters.push_back(meanY);
        xjParameters.push_back(meanZ);
        return true;
    }
    catch (const std::exception&) {
        return false;
    }
}

double calculateQuadrilateralArea(const std::vector<cv::Point2f>& vertices) {
    // 检查顶点数量是否为4
    if (vertices.size() < 3) {
        std::cerr << "错误：顶点数量必须至少为3！" << std::endl;
        return 0.0;
    }

    double area_sum = 0.0;
    int n = vertices.size();

    // 鞋带公式核心计算
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n; // 下一个顶点，最后一个顶点连接到第一个顶点
        area_sum += vertices[i].x * vertices[j].y - vertices[j].x * vertices[i].y;
    }

    // 取绝对值并除以2得到面积
    double area = std::fabs(area_sum) / 2.0;
    return area;
}

bool isPointInRotatedRect(const cv::Point2i& pt, const std::vector<cv::Point2f>& vertices) {
    if (vertices.size() < 3) {
        return false;
    }

    int count = 0;
    int n = vertices.size();

    // 遍历四边形的每条边
    for (int i = 0; i < n; i++) {
        const cv::Point2f& p1 = vertices[i];
        const cv::Point2f& p2 = vertices[(i + 1) % n]; // 下一个顶点，最后一个边连接到第一个顶点

        // 1. 先判断点是否刚好在这条边上
        // 计算点到线段的距离，小于极小值视为在边上
        auto pointToLineSegmentDistance = [](const cv::Point2f& p, const cv::Point2f& a, const cv::Point2f& b) {
            cv::Point2f ab = b - a;
            cv::Point2f ap = p - a;
            double len2 = ab.x * ab.x + ab.y * ab.y;
            if (len2 < 1e-8) { // 线段长度为0
                return sqrt(ap.x * ap.x + ap.y * ap.y);
            }
            double t = std::max(0.0, std::min(1.0, (ap.x * ab.x + ap.y * ab.y) / len2));
            cv::Point2f proj = a + t * ab;
            cv::Point2f diff = p - proj;
            return sqrt(diff.x * diff.x + diff.y * diff.y);
            };

        double dist = pointToLineSegmentDistance(cv::Point2f(pt), p1, p2);
        if (dist < 1e-6) { // 点在边上，直接返回true
            return true;
        }

        // 2. 射线法核心逻辑：判断水平射线与当前边是否相交
        // 确保p1的y坐标 <= p2的y坐标
        cv::Point2f a = p1, b = p2;
        if (a.y > b.y) {
            std::swap(a, b);
        }

        // 跳过射线与边不相交的情况
        if (pt.y < a.y || pt.y > b.y) {
            continue;
        }

        // 计算边与水平射线(y=pt.y)的交点x坐标
        if (std::fabs(a.y - b.y) > 1e-8) { // 边不是水平的
            double x_intersect = (pt.y - a.y) * (b.x - a.x) / (b.y - a.y) + a.x;
            // 交点在点的右侧，计数+1
            if (x_intersect > pt.x) {
                count++;
            }
        }
    }

    // 奇数个交点：点在内部；偶数个：点在外部
    return (count % 2) == 1;
}

ObjectPointDetect::ObjectPointDetect(algorithmParam& param):algorithm_param_global(param){
}
ObjectPointDetect::~ObjectPointDetect() {
}

float computeAngleEigen(const Eigen::Vector3d& n1, const Eigen::Vector3d& n2) {
    float dot_product = n1.dot(n2);
    float norm1 = n1.norm();
    float norm2 = n2.norm();

    dot_product /= (norm1 * norm2);
    dot_product = std::max(-1.0f, std::min(1.0f, dot_product));
    return std::acos(dot_product) * 180.0 / CV_PI;
}

bool segment_cloud_Material_box_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud, pcl::PointCloud<pcl::PointXYZ>& out_cloud, pcl::ModelCoefficients& out_coef) {
    pcl::PointCloud<pcl::PointXYZ> ext_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ext_cloud_rest(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud_cp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*sor_cloud, *sor_cloud_cp); 
    pcl::SACSegmentation<pcl::PointXYZ> sac;
    pcl::PointIndices::Ptr inliner(new  pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new  pcl::ModelCoefficients);
    sac.setInputCloud(sor_cloud);
    sac.setOptimizeCoefficients(true);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setModelType(pcl::SACMODEL_PLANE);
    sac.setMaxIterations(100);
    sac.setDistanceThreshold(20);

    int src_size = sor_cloud->size();
    pcl::ExtractIndices<pcl::PointXYZ> ext;
    bool is_found = 0;

    while (sor_cloud->size() > src_size * 0.1) {
        ext.setInputCloud(sor_cloud);
        sac.segment(*inliner, *coefficients);
        if (inliner->indices.size() == 0)
            break;

        ext.setIndices(inliner);
        ext.setNegative(false);
        ext.filter(ext_cloud);
        ext.setNegative(true);
        ext.filter(*ext_cloud_rest);
        *sor_cloud = *ext_cloud_rest;
        Eigen::Vector3d PtsNormal = Eigen::Vector3d(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

        float ang = computeAngleEigen(PtsNormal, Eigen::Vector3d(0, 0, 1));
        bool found_ang = fabs(ang) < 20 || fabs(ang - 180) < 20;
        out_coef = *coefficients;
        if (1.0 * ext_cloud.size() / sor_cloud_cp->points.size() > 0.6 && found_ang) {
            out_cloud = ext_cloud;
            break;
        }
    }
    return false;
}

bool get_img_mask_obb(cv::Mat& mask, std::vector<cv::Point2f>& mask_pt) {
    std::vector<cv::Point> img_pts;
    uchar* data = mask.data;
    for (int i = 0; i < mask.rows; ++i)
        for (int j = 0; j < mask.cols; ++j) {
            if (data[i * mask.cols + j] > 0) {
                img_pts.emplace_back(j, i);
                break;
            }
        }
    for (int i = 0; i < mask.rows; ++i)
        for (int j = mask.cols - 1; j >= 0; --j) {
            if (data[i * mask.cols + j] > 0) {
                img_pts.emplace_back(j, i);
                break;
            }
        }
    if (img_pts.size() < 20)
        return false;
    cv::RotatedRect min_rect = cv::minAreaRect(img_pts);
    cv::Point2f rect_vertices[4];
    min_rect.points(rect_vertices);
    for (int i = 0; i < 4; ++i)
        mask_pt.push_back(rect_vertices[i]);

    std::sort(mask_pt.begin(), mask_pt.end(), [](const cv::Point2f& p1, const cv::Point2f& p2) {
        return p1.x < p2.x;
        });
    if (mask_pt[0].y < mask_pt[1].y) {
        cv::Point2f tmp = mask_pt[0];
        mask_pt[0] = mask_pt[1];
        mask_pt[1] = tmp;
    }
    if (mask_pt[2].y > mask_pt[3].y) {
        cv::Point2f tmp = mask_pt[2];
        mask_pt[2] = mask_pt[3];
        mask_pt[3] = tmp;
    }

    ////////////////////////////////////////////////////////////
#ifdef debug_show
    cv::Mat result_img;
    cvtColor(mask, result_img, cv::COLOR_GRAY2BGR);
    cv::Scalar rect_color = cv::Scalar(0, 0, 255);  // BGR��ʽ����ɫ
    int thickness = 2;
    for (int i = 0; i < 4; ++i) {
        line(result_img, rect_vertices[i], rect_vertices[(i + 1) % 4], rect_color, thickness, cv::LINE_AA);
    }
    circle(result_img, min_rect.center, 4, cv::Scalar(0, 0, 255), -1);  // -1��ʾ���

    for (int i = 0; i < mask_pt.size(); ++i) {
        circle(result_img, mask_pt[i], 4, cv::Scalar(0, 255, 255), -1);
    }

    //std::cout << "  " << std::endl;
#endif

    return true;
}

void ObjectPointDetect::point_cloud_stat_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>& cloudOut, int MeanK, float STDthre) {
    pcl::PointCloud<pcl::PointXYZ> sor_cloud;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ>sor;
    sor.setMeanK(MeanK);
    sor.setInputCloud(cloudIn);
    sor.setStddevMulThresh(STDthre);
    sor.filter(sor_cloud);
    cloudOut = sor_cloud;
}

class img_point_dist {
public:
    img_point_dist() {
        is_found = 0;
    }
    img_point_dist(cv::Point2f tmp_2d, cv::Point3f tmp_3d, float tmp_dist) {
        pt3d = tmp_3d;
        dist = tmp_dist;
        is_found = 0;
    }
    ~img_point_dist() {
    }
    img_point_dist& operator=(const img_point_dist& tp) {
        if (this != &tp) {
            pt3d = tp.pt3d;
            dist = tp.dist;
            is_found = tp.is_found;
        }
        return *this;
    }
    cv::Point3f pt3d;
    float dist;
    bool is_found = 0;
};

bool get_point_dist(std::vector<img_point_dist>& img_points, img_point_dist& p1, cv::Point3f& center_p3d, img_point_dist& pt_out, bool find_next) {
    float dist = 0;
    float threshold_dist = 25;
    int count = 0;
    center_p3d = cv::Point3f(0, 0, 0);
    bool is_found = 0;
    float min_dist = img_points[img_points.size() - 1].dist;
    float half_min_dist = min_dist / 2.0;
    if (find_next) {
        for (int i = 0; i < img_points.size(); ++i) {
            if (img_points[i].is_found)
                continue;
            dist = sqrt((img_points[i].pt3d.x - p1.pt3d.x) * (img_points[i].pt3d.x - p1.pt3d.x) + (img_points[i].pt3d.y - p1.pt3d.y) * (img_points[i].pt3d.y - p1.pt3d.y) +
                (img_points[i].pt3d.z - p1.pt3d.z) * (img_points[i].pt3d.z - p1.pt3d.z));
            if (dist < threshold_dist) {
                img_points[i].is_found = 1;
                center_p3d.x = center_p3d.x + img_points[i].pt3d.x;
                center_p3d.y = center_p3d.y + img_points[i].pt3d.y;
                center_p3d.z = center_p3d.z + img_points[i].pt3d.z;
                count++;
            }
            else if (dist < half_min_dist)
                img_points[i].is_found = 1;
            //else if (is_found)
            //    break;
            if (dist > min_dist && 0 == is_found) {
                is_found = 1;
                pt_out = img_points[i];
            }
            if (dist > half_min_dist && dist < min_dist)
                break;
        }
    }
    else {
        for (int i = 0; i < img_points.size(); ++i) {
            if (img_points[i].is_found)
                continue;
            dist = sqrt((img_points[i].pt3d.x - p1.pt3d.x) * (img_points[i].pt3d.x - p1.pt3d.x) + (img_points[i].pt3d.y - p1.pt3d.y) * (img_points[i].pt3d.y - p1.pt3d.y) +
                (img_points[i].pt3d.z - p1.pt3d.z) * (img_points[i].pt3d.z - p1.pt3d.z));
            if (dist < threshold_dist) {
                img_points[i].is_found = 1;
                center_p3d.x = center_p3d.x + img_points[i].pt3d.x;
                center_p3d.y = center_p3d.y + img_points[i].pt3d.y;
                center_p3d.z = center_p3d.z + img_points[i].pt3d.z;
                count++;
            }
            //else
            //    break;
            if (dist > half_min_dist && dist < min_dist)
                break;
        }
    }
    if (count < 10)
        return 0;
    if (!is_found && find_next)
        return 0;
    center_p3d.x = center_p3d.x / count;
    center_p3d.y = center_p3d.y / count;
    center_p3d.z = center_p3d.z / count;

    return 1;
}

float dist_point3d(const cv::Point3f& p1, const cv::Point3f& p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

bool ObjectPointDetect::get_material_box_4point(yoloseg_detect_result& box, cv::Mat& xyz, std::vector<float>& intrinsic, std::vector<cv::Point3f>& box_points) {
    if(box.part_mask.empty()){
        LOG_WARN("box.part_mask.empty!!");
        return false;
    }
    std::vector<cv::Point2f> mask_pt;
    if(!get_img_mask_obb(box.part_mask, mask_pt)){
        LOG_WARN("get_img_mask_obb_fail!!");
        return false;
    }
    if (mask_pt.size() != 4){
        LOG_WARN("mask_pt.size()!=4!!  {}",mask_pt.size());
        return false;
    }
    cv::Point2d img_center_pt(0,0);

    float pixel_dxy = 0;
    if (xyz.rows > xyz.cols)
        pixel_dxy = 0.024 * xyz.cols;
    else
        pixel_dxy = 0.024 * xyz.rows;
    for (int i = 0; i < mask_pt.size(); ++i) {
        if (i < 2)
            mask_pt[i].x = mask_pt[i].x + pixel_dxy;
        else
            mask_pt[i].x = mask_pt[i].x - pixel_dxy;
        if (0 == i || 3 == i)
            mask_pt[i].y = mask_pt[i].y - pixel_dxy;
        else
            mask_pt[i].y = mask_pt[i].y + pixel_dxy;
    }

    ///////////////////////////////////////////////
    cv::Mat mark_t = box.part_mask.clone();
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
    int mat_dil_len = 5;
    cv::Mat mark_tp_d = cv::Mat(mark_t.rows + 2 * mat_dil_len, mark_t.cols + 2 * mat_dil_len, CV_8UC1, cv::Scalar(0));
    mark_t.copyTo(mark_tp_d(cv::Rect(mat_dil_len, mat_dil_len, mark_t.cols, mark_t.rows)));
    cv::Mat dilation_img;
    cv::dilate(mark_tp_d, dilation_img, kernel);
    //cv::dilate(dilation_img, dilation_img, kernel);
    //cv::dilate(dilation_img, dilation_img, kernel);

    mark_t = dilation_img.clone();

    //////////////////////////////////////////////
    int col = dilation_img.cols;
    int row = dilation_img.rows;
    float ratio = 0.8;
    int col_t = col * ratio;
    int row_t = row * ratio;
    int start_x = (col - col_t) / 2;
    int start_y = (row - row_t) / 2;
    float ratio_x = 1.0 * col / col_t;
    float ratio_y = 1.0 * row / row_t;
    //mark_t = dilation_img.clone();

    int x_m, y_m;
    uchar* m_data = dilation_img.data;
    uchar* t_data = mark_t.data;
    for (int i = 0; i < row_t; ++i)
        for (int j = 0; j < col_t; ++j) {
            y_m = ratio_x * i;
            x_m = ratio_y * j;
            if (m_data[y_m * col + x_m] > 0) {
                t_data[(i + start_y) * col + j + start_x] = 0;
            }
        }

    int x_start = box.box.x - mat_dil_len;
    int y_start = box.box.y - mat_dil_len;
    pcl::PointCloud<pcl::PointXYZ> points;
    points.reserve(col * row);
    pcl::PointXYZ tmp_p;
    float* xyz_data = (float*)xyz.data;
    int count_i = 0;
    int count_img_pt = 0;
    int xyz_x = 0;
    int xyz_y = 0;
    for (int y = 0; y < row; ++y)
        for (int x = 0; x < col; ++x) {
            if (t_data[y * col + x] > 0) {
                xyz_x = x_start + x;
                xyz_y = y + y_start;
                if (xyz_x >= 0 && xyz_x < xyz.cols && xyz_y >= 0 && xyz_y < xyz.rows) {
                    tmp_p.z = xyz_data[3 * (y + y_start) * xyz.cols + 3 * (x_start + x) + 2];
                    img_center_pt.x = img_center_pt.x + x;
                    img_center_pt.y = img_center_pt.y + y;
                    count_img_pt++;
                    if (tmp_p.z < 10000) {
                        if (count_i % 6 == 0) {
                            tmp_p.x = xyz_data[3 * (y + y_start) * xyz.cols + 3 * (x_start + x)];
                            tmp_p.y = xyz_data[3 * (y + y_start) * xyz.cols + 3 * (x_start + x) + 1];
                            points.push_back(tmp_p);
                        }
                        count_i++;
                    }
                }
            }
        }
    if (points.size() < 100)
        return false;
    std::sort(points.begin(), points.end(), [](pcl::PointXYZ& p1, pcl::PointXYZ& p2) {return p1.z < p2.z; });
    float temp_up_down_z = points[points.size() * 0.7].z;
    pcl::PointCloud<pcl::PointXYZ> top_points;
    top_points.reserve(points.size());
    for (int i = 0; i < points.size(); ++i) {
        if (points[i].z < temp_up_down_z - 50)
            continue;
        top_points.push_back(points[i]);
    }
    //write_point_obj(top_points);
    img_center_pt.x = img_center_pt.x / count_img_pt;
    img_center_pt.y = img_center_pt.y / count_img_pt;

    pcl::PointCloud<pcl::PointXYZ> point_f;
    pcl::PointCloud<pcl::PointXYZ> frame_cloud;
    pcl::ModelCoefficients plane_coef;
    segment_cloud_Material_box_plane(top_points.makeShared(), frame_cloud, plane_coef);
    point_cloud_stat_removal(frame_cloud.makeShared(), point_f, 40, 0.8);

    if (point_f.size() < 100)
        return false;

    ////////////////////////////////////////////////////////////////////////////////////////

    std::vector<cv::Point2f> point_xy;
    for (int i = 0; i < point_f.size(); ++i) {
        point_xy.emplace_back(point_f[i].x, point_f[i].y);
    }
    cv::RotatedRect min_rect = cv::minAreaRect(point_xy);
    cv::Point2f rect_vertices[4];
    min_rect.points(rect_vertices);
    cv::Point3f corner_point[4];
    float dist = 0;
    for (int i = 0; i < 4; ++i) {
        float min_dist = 1000000000;
        for (int j = 0; j < point_f.size(); ++j) {
            dist = ((point_f[j].x - rect_vertices[i].x) * (point_f[j].x - rect_vertices[i].x) + (point_f[j].y - rect_vertices[i].y) * (point_f[j].y - rect_vertices[i].y));
            if (min_dist > dist) {
                min_dist = dist;
                corner_point[i].x = point_f[j].x;
                corner_point[i].y = point_f[j].y;
                corner_point[i].z = point_f[j].z;
            }
        }
    }
    float thre_dist = 25;
    int count = 0;
    std::vector<cv::Point3f> material_box_pts(4);
    for (int i = 0; i < 4; ++i) {
        material_box_pts[i].x = 0;
        material_box_pts[i].y = 0;
        material_box_pts[i].z = 0;
        count = 0;
        for (int j = 0; j < point_f.size(); ++j) { 
            dist = sqrt((point_f[j].x - corner_point[i].x) * (point_f[j].x - corner_point[i].x) + (point_f[j].y - corner_point[i].y) * (point_f[j].y - corner_point[i].y) + (point_f[j].z - corner_point[i].z) * (point_f[j].z - corner_point[i].z));
            if (dist < thre_dist) {
                material_box_pts[i].x += point_f[j].x;
                material_box_pts[i].y += point_f[j].y;
                material_box_pts[i].z += point_f[j].z;
                count++;
            }
        }
        material_box_pts[i].x = material_box_pts[i].x / count;
        material_box_pts[i].y = material_box_pts[i].y / count;
        material_box_pts[i].z = material_box_pts[i].z / count;
    }
    {
        std::vector<cv::Point3f> camera_material_box_pts(material_box_pts.size());
        for (int i = 0; i < material_box_pts.size(); ++i) {
            camera_material_box_pts[i].x = algorithm_param_global.rt_inv[0] * material_box_pts[i].x + algorithm_param_global.rt_inv[1] * material_box_pts[i].y + algorithm_param_global.rt_inv[2] * material_box_pts[i].z + algorithm_param_global.rt_inv[3];
            camera_material_box_pts[i].y = algorithm_param_global.rt_inv[4] * material_box_pts[i].x + algorithm_param_global.rt_inv[5] * material_box_pts[i].y + algorithm_param_global.rt_inv[6] * material_box_pts[i].z + algorithm_param_global.rt_inv[7];
            camera_material_box_pts[i].z = algorithm_param_global.rt_inv[8] * material_box_pts[i].x + algorithm_param_global.rt_inv[9] * material_box_pts[i].y + algorithm_param_global.rt_inv[10] * material_box_pts[i].z + algorithm_param_global.rt_inv[11];
        }

        std::sort(camera_material_box_pts.begin(), camera_material_box_pts.end(), [](cv::Point3f& p1, cv::Point3f& p2) {return p1.x < p2.x; });
        if (camera_material_box_pts[0].y < camera_material_box_pts[1].y) {
            cv::Point3f tmp = camera_material_box_pts[0];
            camera_material_box_pts[0] = camera_material_box_pts[1];
            camera_material_box_pts[1] = tmp;
        }
        if (camera_material_box_pts[2].y > camera_material_box_pts[3].y) {
            cv::Point3f tmp = camera_material_box_pts[2];
            camera_material_box_pts[2] = camera_material_box_pts[3];
            camera_material_box_pts[3] = tmp;
        }
        cv::Point2f reprojection_pt;
        float dist_pt_2f;
        for (int i = 0; i < camera_material_box_pts.size(); ++i) {
            reprojection_pt.x = intrinsic[0] * camera_material_box_pts[i].x / camera_material_box_pts[i].z + intrinsic[2];
            reprojection_pt.y = intrinsic[4] * camera_material_box_pts[i].y / camera_material_box_pts[i].z + intrinsic[5];
            reprojection_pt.x = reprojection_pt.x - box.box.x;
            reprojection_pt.y = reprojection_pt.y - box.box.y;
            dist_pt_2f = std::sqrt((reprojection_pt.x - mask_pt[i].x) * (reprojection_pt.x - mask_pt[i].x) +
                (reprojection_pt.y - mask_pt[i].y) * (reprojection_pt.y - mask_pt[i].y));
            if (dist_pt_2f > 50)
                return false;
        }

        box_points.resize(camera_material_box_pts.size());
        for (int i = 0; i < camera_material_box_pts.size(); ++i) {
            box_points[i].x = algorithm_param_global.camera_rt[0][0] * camera_material_box_pts[i].x + algorithm_param_global.camera_rt[0][1] * camera_material_box_pts[i].y + algorithm_param_global.camera_rt[0][2] * camera_material_box_pts[i].z + algorithm_param_global.camera_rt[0][3];
            box_points[i].y = algorithm_param_global.camera_rt[1][0] * camera_material_box_pts[i].x + algorithm_param_global.camera_rt[1][1] * camera_material_box_pts[i].y + algorithm_param_global.camera_rt[1][2] * camera_material_box_pts[i].z + algorithm_param_global.camera_rt[1][3];
            box_points[i].z = algorithm_param_global.camera_rt[2][0] * camera_material_box_pts[i].x + algorithm_param_global.camera_rt[2][1] * camera_material_box_pts[i].y + algorithm_param_global.camera_rt[2][2] * camera_material_box_pts[i].z + algorithm_param_global.camera_rt[2][3];
            //box_points[i] = corner_point[i];
        }
        if (box_points.size() != 4)
            return false;

        float length1, length2, length3, length4, length5, length6;
        length1 = dist_point3d(box_points[0], box_points[1]);
        length2 = dist_point3d(box_points[1], box_points[2]);
        length3 = dist_point3d(box_points[2], box_points[3]);
        length4 = dist_point3d(box_points[3], box_points[0]);
        length5 = dist_point3d(box_points[0], box_points[2]);
        length6 = dist_point3d(box_points[1], box_points[3]);

        if (fabs(length1 - length3) > 50 || fabs(length2 - length4) > 50
            || fabs(length5 - length6) > 50) {
            return false;
        }
        if(algorithm_param_global.material_box.is_using_box_size){
            if (length1 > length2) {
                if (fabs(length1 - algorithm_param_global.material_box.box_length + 15) > 50 ||
                    fabs(length3 - algorithm_param_global.material_box.box_length + 15) > 50)
                    return false;
                if (fabs(length2 - algorithm_param_global.material_box.box_width + 15) > 50||
                    fabs(length4 - algorithm_param_global.material_box.box_width + 15) > 50)
                    return false;
            }
            else {
                if (fabs(length1 - algorithm_param_global.material_box.box_width + 15) > 50 ||
                    fabs(length3 - algorithm_param_global.material_box.box_width + 15) > 50)
                    return false;
                if (fabs(length2 - algorithm_param_global.material_box.box_length + 15) > 50 ||
                    fabs(length4 - algorithm_param_global.material_box.box_length + 15) > 50)
                    return false;
            }
        }

        //return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////

    cv::Point3d center_pt;
    std::vector<float> plane_normal_base(4);
    plane_normal_base[0] = algorithm_param_global.rt_inv[0] * plane_coef.values[0] + algorithm_param_global.rt_inv[1] * plane_coef.values[1] + algorithm_param_global.rt_inv[2] * plane_coef.values[2];
    plane_normal_base[1] = algorithm_param_global.rt_inv[4] * plane_coef.values[0] + algorithm_param_global.rt_inv[5] * plane_coef.values[1] + algorithm_param_global.rt_inv[6] * plane_coef.values[2];
    plane_normal_base[2] = algorithm_param_global.rt_inv[8] * plane_coef.values[0] + algorithm_param_global.rt_inv[9] * plane_coef.values[1] + algorithm_param_global.rt_inv[10] * plane_coef.values[2];
    plane_normal_base[3] = algorithm_param_global.camera_rt[0][3] * plane_coef.values[0] + algorithm_param_global.camera_rt[1][3] * plane_coef.values[1] + algorithm_param_global.camera_rt[2][3] * plane_coef.values[2] + plane_coef.values[3];
    center_pt.z = -plane_normal_base[3] / (plane_normal_base[0] * (img_center_pt.x - intrinsic[2]) / intrinsic[0] + plane_normal_base[1] * (img_center_pt.y - intrinsic[5]) / intrinsic[4] + plane_normal_base[2]);
    center_pt.x = (img_center_pt.x + x_start - intrinsic[2]) * center_pt.z / intrinsic[0];
    center_pt.y = (img_center_pt.y + y_start - intrinsic[5]) * center_pt.z / intrinsic[4];
    cv::Point3d center_pt_base;
    center_pt_base.x = algorithm_param_global.camera_rt[0][0] * center_pt.x + algorithm_param_global.camera_rt[0][1] * center_pt.y + algorithm_param_global.camera_rt[0][2] * center_pt.z + algorithm_param_global.camera_rt[0][3];
    center_pt_base.y = algorithm_param_global.camera_rt[1][0] * center_pt.x + algorithm_param_global.camera_rt[1][1] * center_pt.y + algorithm_param_global.camera_rt[1][2] * center_pt.z + algorithm_param_global.camera_rt[1][3];
    center_pt_base.z = algorithm_param_global.camera_rt[2][0] * center_pt.x + algorithm_param_global.camera_rt[2][1] * center_pt.y + algorithm_param_global.camera_rt[2][2] * center_pt.z + algorithm_param_global.camera_rt[2][3];

    std::vector<img_point_dist> img_points(point_f.size());

    for (int i = 0; i < img_points.size(); ++i) {
        img_points[i].pt3d.x = point_f[i].x;
        img_points[i].pt3d.y = point_f[i].y;
        img_points[i].pt3d.z = point_f[i].z;
        dist = sqrt((img_points[i].pt3d.x - center_pt_base.x) * (img_points[i].pt3d.x - center_pt_base.x) +
            (img_points[i].pt3d.y - center_pt_base.y) * (img_points[i].pt3d.y - center_pt_base.y) +
            (img_points[i].pt3d.z - center_pt_base.z) * (img_points[i].pt3d.z - center_pt_base.z));
        img_points[i].dist = dist;
    }
    std::sort(img_points.begin(), img_points.end(), [](img_point_dist& p1, img_point_dist& p2){return p1.dist > p2.dist; });

    img_point_dist point4[4];
    std::vector<cv::Point3f> center_p3d(4);
    point4[0] = img_points[0];
    for (int i = 0; i < 3; ++i) {
        if (!get_point_dist(img_points, point4[i], center_p3d[i], point4[i + 1], true))
            return false;
    }
    if (!get_point_dist(img_points, point4[3], center_p3d[3], point4[0], false))
        return false;

    cv::Point3f cam_xyz;
    for (int i = 0; i < center_p3d.size(); ++i) {
        cam_xyz.x = algorithm_param_global.rt_inv[0] * center_p3d[i].x + algorithm_param_global.rt_inv[1] * center_p3d[i].y + algorithm_param_global.rt_inv[2] * center_p3d[i].z + algorithm_param_global.rt_inv[3];
        cam_xyz.y = algorithm_param_global.rt_inv[4] * center_p3d[i].x + algorithm_param_global.rt_inv[5] * center_p3d[i].y + algorithm_param_global.rt_inv[6] * center_p3d[i].z + algorithm_param_global.rt_inv[7];
        cam_xyz.z = algorithm_param_global.rt_inv[8] * center_p3d[i].x + algorithm_param_global.rt_inv[9] * center_p3d[i].y + algorithm_param_global.rt_inv[10] * center_p3d[i].z + algorithm_param_global.rt_inv[11];
        center_p3d[i] = cam_xyz;
    }
    std::sort(center_p3d.begin(), center_p3d.end(), [](cv::Point3f& p1, cv::Point3f& p2) {return p1.x < p2.x; });
    if (center_p3d[0].y < center_p3d[1].y) {
        cv::Point3f tmp = center_p3d[0];
        center_p3d[0] = center_p3d[1];
        center_p3d[1] = tmp;
    }
    if (center_p3d[2].y > center_p3d[3].y) {
        cv::Point3f tmp = center_p3d[2];
        center_p3d[2] = center_p3d[3];
        center_p3d[3] = tmp;
    }
    cv::Point2f reprojection_pt;
    float dist_pt_2f;
    for (int i = 0; i < center_p3d.size(); ++i) {
        reprojection_pt.x = intrinsic[0] * center_p3d[i].x / center_p3d[i].z + intrinsic[2];
        reprojection_pt.y = intrinsic[4] * center_p3d[i].y / center_p3d[i].z + intrinsic[5];
        reprojection_pt.x = reprojection_pt.x - box.box.x;
        reprojection_pt.y = reprojection_pt.y - box.box.y;
        dist_pt_2f = std::sqrt((reprojection_pt.x - mask_pt[i].x) * (reprojection_pt.x - mask_pt[i].x) +
            (reprojection_pt.x - mask_pt[i].x) * (reprojection_pt.x - mask_pt[i].x));
        if (dist_pt_2f > 70)
            return false;
    }

    std::vector<cv::Point3f> box_point_temp;
    box_point_temp.resize(center_p3d.size());
    for (int i = 0; i < center_p3d.size(); ++i) {
        box_point_temp[i].x = algorithm_param_global.camera_rt[0][0] * center_p3d[i].x + algorithm_param_global.camera_rt[0][1] * center_p3d[i].y + algorithm_param_global.camera_rt[0][2] * center_p3d[i].z + algorithm_param_global.camera_rt[0][3];
        box_point_temp[i].y = algorithm_param_global.camera_rt[1][0] * center_p3d[i].x + algorithm_param_global.camera_rt[1][1] * center_p3d[i].y + algorithm_param_global.camera_rt[1][2] * center_p3d[i].z + algorithm_param_global.camera_rt[1][3];
        box_point_temp[i].z = algorithm_param_global.camera_rt[2][0] * center_p3d[i].x + algorithm_param_global.camera_rt[2][1] * center_p3d[i].y + algorithm_param_global.camera_rt[2][2] * center_p3d[i].z + algorithm_param_global.camera_rt[2][3];
    }
    if (box_point_temp.size() != 4)
       return false;

    float length1, length2, length3, length4, length5, length6;
    length1 = dist_point3d(box_point_temp[0], box_point_temp[1]);
    length2 = dist_point3d(box_point_temp[1], box_point_temp[2]);
    length3 = dist_point3d(box_point_temp[2], box_point_temp[3]);
    length4 = dist_point3d(box_point_temp[3], box_point_temp[0]);
    length5 = dist_point3d(box_point_temp[0], box_point_temp[2]);
    length6 = dist_point3d(box_point_temp[1], box_point_temp[3]);

    if (fabs(length1 - length3) > 50 || fabs(length2 - length4) > 50
        || fabs(length5 - length6) > 50) {
        return false;
    }

    if (box_point_temp.size() != box_points.size())
        return false;
    for (int i = 0; i < box_points.size(); ++i) {
        if (dist_point3d(box_points[i], box_point_temp[i]) > 50)
            return false;
    }

    return true;
}

pcl::Normal computeAverageNormal(const std::vector<pcl::Normal>& normals) {
    if (normals.empty()) {
        return pcl::Normal(0, 0, 0);
    }

    // 1. 选择第一个法向量作为参考（也可选择重心法向量/随机向量）
    Eigen::Vector3d ref_normal(normals[0].normal_x, normals[0].normal_y, normals[0].normal_z);

    // 2. 初始化累加向量
    Eigen::Vector3d sum_normal(0.0, 0.0, 0.0);

    // 3. 遍历法向量，统一方向后累加
    for (const auto& n : normals) {
        Eigen::Vector3d curr_normal(n.normal_x, n.normal_y, n.normal_z);
        // 点积判断方向，负则翻转
        double dot = ref_normal.dot(curr_normal);
        if (dot < 0) {
            curr_normal = -curr_normal;
        }
        sum_normal += curr_normal;
    }

    // 4. 归一化（避免零向量，容错处理）
    double norm = sum_normal.norm();
    if (norm < 1e-6f) {
        return pcl::Normal(0, 0, 0);
    }
    sum_normal.normalize();

    // 5. 转换为PCL法向量
    return pcl::Normal(sum_normal.x(), sum_normal.y(), sum_normal.z());
}

void calculateEulerAnglesFromNormal(const pcl::Normal& normal, float& pitch, float& roll) {
    // 确保法向量是单位向量
    Eigen::Vector3f normalized_normal(normal.normal_x, normal.normal_y, normal.normal_z);
    normalized_normal.normalize();

    // 计算俯仰角 (Pitch) = arcsin(-ny)
    pitch = std::asin(-normalized_normal.y()) * 180.0 / M_PI;

    // 计算滚转角 (Roll) = atan2(nx, nz)
    roll = std::atan2(normalized_normal.x(), normalized_normal.z()) * 180.0 / M_PI;
}

void get_points_area(std::vector<cv::Point2f>& points, pcl::Normal& normal, float& area, float& rect_area, cv::RotatedRect& min_rect) {
    // 2. 计算凸包（得到有序轮廓）
    std::vector<cv::Point2f> hull_points;
    convexHull(cv::Mat(points), hull_points); // 直接输出凸包顶点（无需索引转换）

    // 3. 直接调用 contourArea 计算面积（核心！）
    double area2d = contourArea(hull_points);
    Eigen::Vector3d normal_plane(normal.normal_x, normal.normal_y, normal.normal_z);
    Eigen::Vector3d normal_z(0, 0, 1);
    //float ang = computeAngleEigen(normal_e, Eigen::Vector3d(0, 0, 1)); 
    float dot_product = normal_plane.dot(normal_z);
    float norm1 = normal_plane.norm();
    float norm2 = normal_z.norm();

    dot_product /= (norm1 * norm2);
    dot_product = std::max(-1.0f, std::min(1.0f, dot_product));
    area = area2d / dot_product;
    min_rect = cv::minAreaRect(points);
    rect_area = min_rect.size.area();
    rect_area = rect_area / dot_product;

}

Eigen::Matrix3d getRotationMatrix(double angleX_deg, double angleY_deg, double angleZ_deg) {
    // 角度转弧度
    double rx = angleX_deg * M_PI / 180.0;
    double ry = angleY_deg * M_PI / 180.0;
    double rz = angleZ_deg * M_PI / 180.0;

    // 1. 绕X轴的旋转矩阵 Rx
    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
        0, cos(rx), -sin(rx),
        0, sin(rx), cos(rx);

    // 2. 绕Y轴的旋转矩阵 Ry
    Eigen::Matrix3d Ry;
    Ry << cos(ry), 0, sin(ry),
        0, 1, 0,
        -sin(ry), 0, cos(ry);

    // 3. 绕Z轴的旋转矩阵 Rz
    Eigen::Matrix3d Rz;
    Rz << cos(rz), -sin(rz), 0,
        sin(rz), cos(rz), 0,
        0, 0, 1;

    // 组合旋转矩阵：R = Rz * Ry * Rx（与你原lambda函数的旋转顺序一致）
    //Matrix3d R = Rz * Ry * Rx;
    Eigen::Matrix3d R = Rx * Ry * Rz;
    return R;
}
void rotationMatrixToXyzAngles(const Eigen::Matrix3d& R, double& new_angleX_deg,
    double& new_angleY_deg, double& new_angleZ_deg) {
    double rx, ry, rz;

    // ******** 修正核心：正确的XYZ欧拉角分解公式 ********
    // 步骤1：分解绕Y轴的角度（俯仰角）- 关键修正：用R(2,0)而非R(0,2)
    ry = asin(-R(2, 0));

    // 处理奇异情况（万向锁：ry=±90°）
    if (abs(ry) > M_PI / 2 - 1e-6) {
        rx = 0.0;  // 横滚角置0
        ry = M_PI / 2; // 固定为90°
        // 分解绕Z轴的角度
        rz = atan2(R(0, 1), R(1, 1));
    }
    else {
        // 步骤2：分解绕X轴的角度（横滚角）- 修正行列索引
        rx = atan2(R(2, 1), R(2, 2));
        // 步骤3：分解绕Z轴的角度（偏航角）- 修正行列索引
        rz = atan2(R(1, 0), R(0, 0));
    }

    // 弧度转角度
    new_angleX_deg = rx * 180.0 / M_PI;
    new_angleY_deg = ry * 180.0 / M_PI;
    new_angleZ_deg = rz * 180.0 / M_PI;
}

struct OBB3D {
    // 包围盒中心
    cv::Point3d center;

    // 三个正交方向轴
    Eigen::Vector3d axis_x;
    Eigen::Vector3d axis_y;
    Eigen::Vector3d axis_z;

    // 半长宽高
    double half_extent_x;
    double half_extent_y;
    double half_extent_z;

    // 最终长宽高
    double width() { return 2 * half_extent_x; }
    double height() { return 2 * half_extent_y; }
    double depth() { return 2 * half_extent_z; }

    // 8个顶点（OpenCV 格式）
    std::vector<cv::Point3d> vertices;
};

// 计算 3D 点云最小外包包围盒（输入：OpenCV cv::Point3d）
OBB3D computeOBB3D(const std::vector<cv::Point3d>& points) {
    OBB3D obb;
    int n = points.size();
    if (n == 0) return obb;

    // ===================== 1. 计算点云中心 =====================
    double cx = 0, cy = 0, cz = 0;
    for (const auto& p : points) {
        cx += p.x;
        cy += p.y;
        cz += p.z;
    }
    cx /= n;
    cy /= n;
    cz /= n;
    obb.center = cv::Point3d(cx, cy, cz);

    // ===================== 2. 构建 3x3 协方差矩阵 =====================
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : points) {
        double dx = p.x - cx;
        double dy = p.y - cy;
        double dz = p.z - cz;

        cov(0, 0) += dx * dx;
        cov(0, 1) += dx * dy;
        cov(0, 2) += dx * dz;

        cov(1, 0) += dy * dx;
        cov(1, 1) += dy * dy;
        cov(1, 2) += dy * dz;

        cov(2, 0) += dz * dx;
        cov(2, 1) += dz * dy;
        cov(2, 2) += dz * dz;
    }
    cov /= n;

    // ===================== 3. 特征值分解（PCA） =====================
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d ax = solver.eigenvectors().col(2);
    Eigen::Vector3d ay = solver.eigenvectors().col(1);
    Eigen::Vector3d az = solver.eigenvectors().col(0);

    obb.axis_x = ax;
    obb.axis_y = ay;
    obb.axis_z = az;

    // ===================== 4. 投影到三个轴求极值 =====================
    double min_x = 1e9, max_x = -1e9;
    double min_y = 1e9, max_y = -1e9;
    double min_z = 1e9, max_z = -1e9;

    for (const auto& p : points) {
        double dx = p.x - cx;
        double dy = p.y - cy;
        double dz = p.z - cz;

        double px = dx * ax.x() + dy * ax.y() + dz * ax.z();
        double py = dx * ay.x() + dy * ay.y() + dz * ay.z();
        double pz = dx * az.x() + dy * az.y() + dz * az.z();

        min_x = std::min(min_x, px);
        max_x = std::max(max_x, px);
        min_y = std::min(min_y, py);
        max_y = std::max(max_y, py);
        min_z = std::min(min_z, pz);
        max_z = std::max(max_z, pz);
    }

    obb.half_extent_x = (max_x - min_x) / 2.0;
    obb.half_extent_y = (max_y - min_y) / 2.0;
    obb.half_extent_z = (max_z - min_z) / 2.0;

    // ===================== 5. 计算 8 个顶点（OpenCV 格式） =====================
    Eigen::Vector3d c(cx, cy, cz);
    std::vector<Eigen::Vector3d> corners = {
        c - ax * obb.half_extent_x - ay * obb.half_extent_y - az * obb.half_extent_z,
        c + ax * obb.half_extent_x - ay * obb.half_extent_y - az * obb.half_extent_z,
        c + ax * obb.half_extent_x + ay * obb.half_extent_y - az * obb.half_extent_z,
        c - ax * obb.half_extent_x + ay * obb.half_extent_y - az * obb.half_extent_z,

        c - ax * obb.half_extent_x - ay * obb.half_extent_y + az * obb.half_extent_z,
        c + ax * obb.half_extent_x - ay * obb.half_extent_y + az * obb.half_extent_z,
        c + ax * obb.half_extent_x + ay * obb.half_extent_y + az * obb.half_extent_z,
        c - ax * obb.half_extent_x + ay * obb.half_extent_y + az * obb.half_extent_z,
    };

    for (auto& cor : corners) {
        obb.vertices.emplace_back(cor.x(), cor.y(), cor.z());
    }

    return obb;
}


bool ObjectPointDetect::calculate_object_pose(yoloseg_detect_result& object_seg, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<cv::Point3d>& center_pt, std::vector < cv::Point3f>& angle_plane, std::vector<int>& points_size,std::vector<cv::Point3i>& plane_size, std::vector<cv::Point3f>& polygon_vertex) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setKSearch(30);
    normal_estimator.setViewPoint(0.0, 0.0, 10000.0); //
    normal_estimator.compute(*normals);

    int planar_count = 0;
    for (auto& n : normals->points) {
        if (n.curvature < 0.03) planar_count++;
    }
    double plane_ratio = planar_count / (double)cloud->points.size();

    bool is_plane_seg = 0;
    pcl::Normal plane_normal_seg;
    std::vector<pcl::PointIndices> clusters;
    if (plane_ratio > 0.5) {
        is_plane_seg = 1;
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(5.0);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coeffs);

        plane_normal_seg.normal_x = coeffs->values[0];
        plane_normal_seg.normal_y = coeffs->values[1];
        plane_normal_seg.normal_z = coeffs->values[2];
        if (plane_normal_seg.normal_z < 0) {
            plane_normal_seg.normal_x = -plane_normal_seg.normal_x;
            plane_normal_seg.normal_y = -plane_normal_seg.normal_y;
            plane_normal_seg.normal_z = -plane_normal_seg.normal_z;
        }
        pcl::PointIndices temp_ind;
        float dist = 0;

        for (int i = 0; i < cloud->size(); ++i) {
            dist = coeffs->values[0] * cloud->points[i].x + coeffs->values[1] * cloud->points[i].y +
                coeffs->values[2] * cloud->points[i].z + coeffs->values[3];
            if (fabs(dist) < 11) {
                    temp_ind.indices.push_back(i);
            }
        }
        clusters.push_back(temp_ind);
    }
    else {
        // 创建区域生长分割对象
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setInputCloud(cloud);
        reg.setInputNormals(normals);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(100);
        reg.setSmoothnessThreshold(6 * 3.14159 / 180.0);    // 法线夹角阈值
        reg.setCurvatureThreshold(0.1);        // 曲率阈值  1.0
        reg.setMinClusterSize(120);
        reg.setMaxClusterSize(1000000);
        // 执行分割
        reg.extract(clusters);
    }
   
    ///////////////////////////////////////////////////
    if (clusters.size() < 1)
        return 0;

    if (clusters[0].indices.size() < 100)
        return 0;

    cv::Point3i temp_plane_size;
    cv::Point3d center_pt_tp(0, 0, 0);
    std::vector<pcl::Normal> plane_normals;
    std::vector<cv::Point2f> points_2d;
    pcl::PointCloud<pcl::PointXYZ> plane_points;
    int max_index = 0;
    int max_point_size = 0;
    for (int i = 0; i < clusters.size(); ++i) {
        if (max_point_size < clusters[i].indices.size()) {
            max_index = i;
            max_point_size = clusters[i].indices.size();
        }
    }
    //for (int i = 0; i < clusters.size(); ++i) {
    {
        int i = max_index;
        plane_normals.clear();
        points_2d.clear();
        plane_points.clear();
        plane_normals.resize(clusters[i].indices.size());
        std::vector<cv::Point3d> points3d(clusters[i].indices.size());
        for (int j = 0; j < clusters[i].indices.size(); ++j) {
            points3d[j].x = cloud->points[clusters[i].indices[j]].x;
            points3d[j].y = cloud->points[clusters[i].indices[j]].y;
            points3d[j].z = cloud->points[clusters[i].indices[j]].z;

            plane_normals[j] = normals->points[clusters[i].indices[j]];
            center_pt_tp.x = center_pt_tp.x + cloud->points[clusters[i].indices[j]].x;
            center_pt_tp.y = center_pt_tp.y + cloud->points[clusters[i].indices[j]].y;
            center_pt_tp.z = center_pt_tp.z + cloud->points[clusters[i].indices[j]].z;
            points_2d.emplace_back(cloud->points[clusters[i].indices[j]].x, cloud->points[clusters[i].indices[j]].y);
            plane_points.push_back(cloud->points[clusters[i].indices[j]]);
        }

        center_pt_tp.x = center_pt_tp.x / clusters[i].indices.size();
        center_pt_tp.y = center_pt_tp.y / clusters[i].indices.size();
        center_pt_tp.z = center_pt_tp.z / clusters[i].indices.size();
        pcl::Normal average_normal = computeAverageNormal(plane_normals);
        float r_x, r_y;
        if (is_plane_seg) {
            calculateEulerAnglesFromNormal(plane_normal_seg, r_x, r_y);
        }
        else {
            calculateEulerAnglesFromNormal(average_normal, r_x, r_y);
        }
        center_pt.push_back(center_pt_tp);

        OBB3D temp_obb3d =  computeOBB3D(points3d);
        float area_3d = 0;
        float area_rect_3d = 0;
        cv::RotatedRect temp_rect;
        get_points_area(points_2d, average_normal, area_3d, area_rect_3d, temp_rect);
        std::vector<cv::Point2f> convex_hull;
        convexHull(points_2d, convex_hull);
        
        temp_plane_size.z = temp_obb3d.depth();
        float temp_angle;
        if (temp_rect.size.width < temp_rect.size.height) {
            temp_angle = temp_rect.angle;
            temp_plane_size.x = temp_rect.size.height;
            temp_plane_size.y = temp_rect.size.width;
        }
        else {
            temp_plane_size.x = temp_rect.size.width;
            temp_plane_size.y = temp_rect.size.height;
            temp_angle = temp_rect.angle - 90;
        }
        Eigen::Matrix3d rotationMatrix = getRotationMatrix(r_x, r_y, temp_angle);
        double new_angleX_deg, new_angleY_deg, new_angleZ_deg;
        rotationMatrixToXyzAngles(rotationMatrix, new_angleX_deg, new_angleY_deg, new_angleZ_deg);
        //angle_plane.emplace_back(r_x, r_y,temp_angle);
        angle_plane.emplace_back(new_angleX_deg, new_angleY_deg,new_angleZ_deg);
        LOG_WARN("seg_plane_angle!!!  {}  {}  {} | {}  {}  {} ", r_x, r_y, temp_angle, new_angleX_deg, new_angleY_deg, new_angleZ_deg);
        ///////////////////////////////////////////////real_area
        std::vector<double> pParam_plane;
        fit_plane_by_least_squares(pParam_plane, plane_points.makeShared());
        /////////////////////////////
        double plane_in_camera[4];
        plane_in_camera[0] = pParam_plane[0] * algorithm_param_global.camera_rt[0][0] + pParam_plane[1] * algorithm_param_global.camera_rt[1][0] + pParam_plane[2] * algorithm_param_global.camera_rt[2][0];
        plane_in_camera[1] = pParam_plane[0] * algorithm_param_global.camera_rt[0][1] + pParam_plane[1] * algorithm_param_global.camera_rt[1][1] + pParam_plane[2] * algorithm_param_global.camera_rt[2][1];
        plane_in_camera[2] = pParam_plane[0] * algorithm_param_global.camera_rt[0][2] + pParam_plane[1] * algorithm_param_global.camera_rt[1][2] + pParam_plane[2] * algorithm_param_global.camera_rt[2][2];
        plane_in_camera[3] = pParam_plane[0] * algorithm_param_global.camera_rt[0][3] + pParam_plane[1] * algorithm_param_global.camera_rt[1][3] + pParam_plane[2] * algorithm_param_global.camera_rt[2][3] + pParam_plane[3];
        if (pParam_plane.size() > 3) {
            std::vector<cv::Point2f> rect_vertices;
            cv::approxPolyDP(convex_hull, rect_vertices, 10, true);

            int img_expand_size = 2;
            cv::Mat pt_area = cv::Mat(object_seg.part_mask.rows + img_expand_size * 2, object_seg.part_mask.cols + img_expand_size * 2, CV_8UC1, cv::Scalar(0));
            uchar* pt_area_data = pt_area.data;

            float temp_point_z;
            std::vector<cv::Point2f> pixel_rect_vertices;
            std::vector<cv::Point2f> xy_rect_vertices;
            cv::Point3f point3d_tmp;
            cv::Point2f rect_pixel;
            for (int i = 0; i < rect_vertices.size(); ++i) {
                temp_point_z = (-pParam_plane[3] - rect_vertices[i].x * pParam_plane[0] - rect_vertices[i].y * pParam_plane[1]) / pParam_plane[2];
                point3d_tmp.x = algorithm_param_global.rt_inv[0] * rect_vertices[i].x + algorithm_param_global.rt_inv[1] * rect_vertices[i].y + algorithm_param_global.rt_inv[2] * temp_point_z + algorithm_param_global.rt_inv[3];
                point3d_tmp.y = algorithm_param_global.rt_inv[4] * rect_vertices[i].x + algorithm_param_global.rt_inv[5] * rect_vertices[i].y + algorithm_param_global.rt_inv[6] * temp_point_z + algorithm_param_global.rt_inv[7];
                point3d_tmp.z = algorithm_param_global.rt_inv[8] * rect_vertices[i].x + algorithm_param_global.rt_inv[9] * rect_vertices[i].y + algorithm_param_global.rt_inv[10] * temp_point_z + algorithm_param_global.rt_inv[11];
                rect_pixel.x = algorithm_param_global.cam_data.intrinsic[0] * point3d_tmp.x / point3d_tmp.z + algorithm_param_global.cam_data.intrinsic[2];
                rect_pixel.y = algorithm_param_global.cam_data.intrinsic[4] * point3d_tmp.y / point3d_tmp.z + algorithm_param_global.cam_data.intrinsic[5];
                pixel_rect_vertices.push_back(rect_pixel);
                xy_rect_vertices.emplace_back(point3d_tmp.x, point3d_tmp.y);
            }
            double pixel_rect_area = calculateQuadrilateralArea(pixel_rect_vertices);
            float area_ratio = area_rect_3d / pixel_rect_area;
            uchar* m_data = object_seg.part_mask.data;
            int col = object_seg.part_mask.cols;
            int row = object_seg.part_mask.rows;
            int x_start = object_seg.box.x;
            int y_start = object_seg.box.y;
            pcl::PointXYZ tmp_p;
            bool is_in_rect;
            cv::Point2i pixel_seg;
            double pixel_area = 0;
            float temp_1;
            float temp_2;
            float point_z;
            cv::Point2f point_xy;
            for (int y = 0; y < row; y++)
                for (int x = 0; x < col; x++) {
                    if (m_data[y * col + x] > 0) {
                        pixel_seg.x = x_start + x;
                        pixel_seg.y = y_start + y;
                        is_in_rect = isPointInRotatedRect(pixel_seg, pixel_rect_vertices);
                        if (is_in_rect) {
                            pixel_area++;
                        }

                        ///////////////////////////////////
                        temp_1 = plane_in_camera[0] * (pixel_seg.x - algorithm_param_global.cam_data.intrinsic[2]) / algorithm_param_global.cam_data.intrinsic[0];
                        temp_2 = plane_in_camera[1] * (pixel_seg.y - algorithm_param_global.cam_data.intrinsic[5]) / algorithm_param_global.cam_data.intrinsic[4];
                        point_z = (-plane_in_camera[3]) / (temp_1 + temp_2 + plane_in_camera[2]);
                        point_xy.x = point_z * (pixel_seg.x - algorithm_param_global.cam_data.intrinsic[2]) / algorithm_param_global.cam_data.intrinsic[0];
                        point_xy.y = point_z * (pixel_seg.y - algorithm_param_global.cam_data.intrinsic[5]) / algorithm_param_global.cam_data.intrinsic[4];
                        is_in_rect = isPointInRotatedRect(point_xy, xy_rect_vertices);
                        if (is_in_rect) {
                            pt_area_data[(y + img_expand_size) * pt_area.cols + x + img_expand_size] = 255;
                        }
                    }
                }
            LOG_WARN("seg_plane_from_norm_eara  {}  {}", area_3d, pixel_area * area_ratio);
            area_3d = pixel_area * area_ratio;
            /////////////////////////////
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(pt_area, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            if (contours.size() < 1)
                return 0;
            int index_contour = 0;
            int max_contour_size = 0;
            for (int i = 0; i < contours.size(); ++i) {
                if (contours[i].size() > max_contour_size) {
                    max_contour_size = contours[i].size();
                    index_contour = i;
                }
            }
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contours[index_contour], approx, 4, true);
            //////////////////////////////////////////////////////
            //cv::Mat img_show;
            //cv::cvtColor(pt_area, img_show, cv::COLOR_GRAY2BGR);
            //for (int i = 0; i < approx.size(); ++i) {
            //    cv::line(img_show, approx[i], approx[(i + 1) % approx.size()], cv::Scalar(0, 0, 255), 2);
            //}
            /////////////////////////////////////
            cv::Point3f temp_vertex;
            for (int i = 0; i < approx.size(); ++i) {
                approx[i].x = approx[i].x - img_expand_size + x_start;
                approx[i].y = approx[i].y - img_expand_size + y_start;
                temp_1 = plane_in_camera[0] * (approx[i].x - algorithm_param_global.cam_data.intrinsic[2]) / algorithm_param_global.cam_data.intrinsic[0];
                temp_2 = plane_in_camera[1] * (approx[i].y - algorithm_param_global.cam_data.intrinsic[5]) / algorithm_param_global.cam_data.intrinsic[4];
                point_z = (-plane_in_camera[3]) / (temp_1 + temp_2 + plane_in_camera[2]);
                point_xy.x = point_z * (approx[i].x - algorithm_param_global.cam_data.intrinsic[2]) / algorithm_param_global.cam_data.intrinsic[0];
                point_xy.y = point_z * (approx[i].y - algorithm_param_global.cam_data.intrinsic[5]) / algorithm_param_global.cam_data.intrinsic[4];


                temp_vertex.x = algorithm_param_global.camera_rt[0][0] * point_xy.x + algorithm_param_global.camera_rt[0][1] * point_xy.y + algorithm_param_global.camera_rt[0][2] * point_z + algorithm_param_global.camera_rt[0][3];
                temp_vertex.y = algorithm_param_global.camera_rt[1][0] * point_xy.x + algorithm_param_global.camera_rt[1][1] * point_xy.y + algorithm_param_global.camera_rt[1][2] * point_z + algorithm_param_global.camera_rt[1][3];
                temp_vertex.z = algorithm_param_global.camera_rt[2][0] * point_xy.x + algorithm_param_global.camera_rt[2][1] * point_xy.y + algorithm_param_global.camera_rt[2][2] * point_z + algorithm_param_global.camera_rt[2][3];
                polygon_vertex.push_back(temp_vertex);
            }
        }
        else
            return 0;
        int area_int = area_3d;
        points_size.push_back(area_int);
        plane_size.push_back(temp_plane_size);
    }
#ifdef debug_show
    {
        points_norm_show(*cloud, *normals);
        ///////////////////////////////
        std::vector<pcl::PointCloud<pcl::PointXYZ>> points_v;
        std::vector<pcl::PointCloud<pcl::Normal>> normals_v;
        points_v.push_back(*cloud);
        pcl::PointCloud<pcl::PointXYZ> temp;
        pcl::PointCloud<pcl::Normal> temp_norm;
        std::vector<cv::Point2f> point_x_y;

        for (int i = 0; i < clusters.size(); ++i) {
            temp.resize(clusters[i].indices.size());
            temp_norm.resize(clusters[i].indices.size());
            for (int j = 0; j < clusters[i].indices.size(); ++j) {
                temp[j] = cloud->points[clusters[i].indices[j]];
                temp_norm[j] = normals->points[clusters[i].indices[j]];
            }
            points_v.push_back(temp);
            temp.clear();
            normals_v.push_back(temp_norm);
            temp_norm.clear();
        }

        points_viewer_show(points_v);
        std::cout << "-----------" << std::endl;
    }
#endif // debug_show

    return 1;
}

bool ObjectPointDetect::is_object_above_material_box(yoloseg_detect_result& box, cv::Mat& xyz) {
    if (box.part_mask.empty()) {
        LOG_ERROR("is_object_above_material_box_box.part_mask.empty!!");
        return false;
    }

    if (algorithm_param_global.material_box.material_box_points.size() != 4) {
        LOG_ERROR("is_object_above_material_box_box_material_box_points.size !=4!!");
        return false;
    }

    int limit_z = -10000000;
    std::vector<cv::Point2f> material_point2f(4);
    for (int i = 0; i < algorithm_param_global.material_box.material_box_points.size(); ++i) {
        material_point2f[i].x = algorithm_param_global.material_box.material_box_points[i].x;
        material_point2f[i].y = algorithm_param_global.material_box.material_box_points[i].y;
        if (limit_z < algorithm_param_global.material_box.material_box_points[i].z) {
            limit_z = algorithm_param_global.material_box.material_box_points[i].z;
        }
    }
    limit_z = limit_z + 100;
    
    int col = box.part_mask.cols;
    int row = box.part_mask.rows;
    int x_start = box.box.x;
    int y_start = box.box.y;
    uchar* mark_data = box.part_mask.data;
    float* xyz_data = (float*)xyz.data;
    int xyz_x = 0;
    int xyz_y = 0;
    pcl::PointXYZ tmp_p;
    bool inside = false;
    int count_out_z = 0;
    int count_all = 0;
    //pcl::PointCloud<pcl::PointXYZ> top_pts;
    //pcl::PointCloud<pcl::PointXYZ> points;
    for (int y = 0; y < row; ++y)
        for (int x = 0; x < col; ++x) {
            if (mark_data[y * col + x] > 0) {
                xyz_x = x_start + x;
                xyz_y = y + y_start;
                if (xyz_x >= 0 && xyz_x < xyz.cols && xyz_y >= 0 && xyz_y < xyz.rows) {
                    tmp_p.z = xyz_data[3 * (y + y_start) * xyz.cols + 3 * (x_start + x) + 2];
                    if (tmp_p.z < 10000) {
                        tmp_p.x = xyz_data[3 * (y + y_start) * xyz.cols + 3 * (x_start + x)];
                        tmp_p.y = xyz_data[3 * (y + y_start) * xyz.cols + 3 * (x_start + x) + 1];
                        
                        inside = false;
                        for (int ik = 0, jk = 3; ik < 4; jk = ik++)
                        {
                            const cv::Point2f& vi = material_point2f[ik];
                            const cv::Point2f& vj = material_point2f[jk];
                            if (((vi.y > tmp_p.y) != (vj.y > tmp_p.y)) &&
                                (tmp_p.x < (vj.x - vi.x) * (tmp_p.y - vi.y) / (vj.y - vi.y) + vi.x))
                            {
                                inside = !inside;
                            }
                        }
                        if (inside && tmp_p.z > limit_z) {
                            count_out_z++;
                            //top_pts.push_back(tmp_p);
                        }
                        else {
                            count_all++;
                            //points.push_back(tmp_p);
                        }
                    }
                }
            }
        }

    std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud;
    //cloud.push_back(points);
    //cloud.push_back(top_pts);
    //points_viewer_show(cloud);
    float ratio = 1.0 * count_out_z / count_all;
    int pixel_number = col * row;
    float ratio_pixel = 1.0 * count_all / pixel_number;
    LOG_INFO("pixel_number_ratio_pixel {}  {}", pixel_number, ratio_pixel);
    if (ratio > 0.1 || ratio_pixel < 0.4)
        return false;
    return true;
}