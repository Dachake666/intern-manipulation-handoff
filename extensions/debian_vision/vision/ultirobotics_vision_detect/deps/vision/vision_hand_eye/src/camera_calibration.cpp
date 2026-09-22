#include"camera_calibration.h"
#include <fstream>
#include <regex>
//#include <io.h>
#include <random>
#include <pcl/segmentation/region_growing.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#define m_PI 3.14159265358979323846

std::string get_compact_matrix_string(const nlohmann::json& matrix) {
    std::string matrix_str = "[\n";  // 开头换行
    for (size_t i = 0; i < matrix.size(); ++i) {
        const auto& row = matrix[i];
        matrix_str += "    [";  // 4空格缩进（匹配JSON整体缩进）
        for (size_t j = 0; j < row.size(); ++j) {
            matrix_str += row[j].dump();
            // 最后一列不加空格，其他列后加逗号（0.0000后保留一个空格）
            if (j != row.size() - 1) {
                // 对0.0000特殊处理：保留一个空格（和示例一致）
                if (row[j] == 0.0 && j >= row.size() - 4) {
                    matrix_str += ", ";
                } else {
                    matrix_str += ",";
                }
            }
        }
        matrix_str += "]";
        // 最后一行不加逗号，其他行加
        if (i != matrix.size() - 1) {
            matrix_str += ",";
        }
        matrix_str += "\n";  // 每行结束换行
    }
    matrix_str += "    ]";  // 结尾4空格缩进
    return matrix_str;
}


// 1. ��תƫ�����ת��ת���Ƕȡ㣩
double rotMat2Angle(const cv::Mat& R_err) {
    // ��ת����ļ� �� ��ת�ǣ����ȣ��� ת��Ϊ�Ƕ�
    double tr = cv::trace(R_err)[0];
    double theta_rad = std::acos((tr - 1) / 2.0);
    // ������ֵ�����acos��������[-1,1]��
    if (std::isnan(theta_rad)) theta_rad = 0.0;
    return theta_rad * 180.0 / CV_PI;
}

// 2. ����ƽ����ŷ�Ͼ��룬��λ��mm���޸�convertTo����void�����⣩
double calcTransError(const cv::Mat& t_true, const cv::Mat& t_cal) {
    // ����1���ȿ�¡������ת�����ͣ��ֿ�д������void��ֵ��
    cv::Mat t1 = t_true.clone();  // ��¡ԭ����
    t1.convertTo(t1, CV_64F);    // ת��ΪCV_64F���ͣ�����void���������ã�

    cv::Mat t2 = t_cal.clone();   // ��¡ԭ����
    t2.convertTo(t2, CV_64F);     // ת��ΪCV_64F����

    // ����ŷ�Ͼ���
    return cv::norm(t1 - t2, cv::NORM_L2);
}

// 3. �����ת��ƽ��Ϊ4��4��ξ���
cv::Mat composeHomogeneousMat(const cv::Mat& R, const cv::Mat& t) {
    cv::Mat H = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(H(cv::Rect(0, 0, 3, 3)));
    t.copyTo(H(cv::Rect(3, 0, 1, 3)));
    return H;
}

// 4. ����ξ�����棨���Ա任��������м򻯹�ʽ����ͨ��������죩
cv::Mat rigidMatInverse(const cv::Mat& H) {
    cv::Mat H_inv = cv::Mat::eye(4, 4, CV_64F);
    // ��ת���֣���=ת�ã�ƽ�Ʋ��֣�-R^T * t
    cv::Mat R = H(cv::Rect(0, 0, 3, 3)).t();
    cv::Mat t = H(cv::Rect(3, 0, 1, 3));
    cv::Mat t_inv = -R * t;

    R.copyTo(H_inv(cv::Rect(0, 0, 3, 3)));
    t_inv.copyTo(H_inv(cv::Rect(3, 0, 1, 3)));
    return H_inv;
}

// ���룺�궨�������������� + �궨���R_cam2base/t_cam2base
// �������ת����б���ƽ������б������ں���ͳ�ƣ�
void calcHandEyeError_EyeToHand(
    const std::vector<cv::Mat>& R_base2gripper,  // N�� ���������� ��ת����
    const std::vector<cv::Mat>& T_base2gripper,  // N�� ���������� ƽ������
    const std::vector<cv::Mat>& R_target2cam,    // N�� �궨������ ��ת����
    const std::vector<cv::Mat>& T_target2cam,    // N�� �궨������ ƽ������
    const cv::Mat& R_cam2base,                   // �궨�������������� ��ת
    const cv::Mat& t_cam2base,                   // �궨�������������� ƽ��
    std::vector<double>& rot_errors_deg,         // �����ÿ�����ת���㣩
    std::vector<double>& trans_errors_mm         // �����ÿ���ƽ����mm��
) {
    // ��ʼ������б�
    rot_errors_deg.clear();
    trans_errors_mm.clear();
    int N = R_base2gripper.size();
    if (N < 2) {
        std::cerr << "���������㣬������Ҫ2�����ݣ�" << std::endl;
        return;
    }

    // ����1�������������������ξ��� H_cam2base
    cv::Mat H_cam2base = composeHomogeneousMat(R_cam2base, t_cam2base);

    // ����2�������0�������� H_gripper2target_0����Ϊ�ο���ֵ��
    cv::Mat H_base2gripper_0 = composeHomogeneousMat(R_base2gripper[0], T_base2gripper[0]);
    cv::Mat H_target2cam_0 = composeHomogeneousMat(R_target2cam[0], T_target2cam[0]);
    //cv::Mat H_gripper2target_0 = rigidMatInverse(H_base2gripper_0) * H_cam2base * H_target2cam_0;
    cv::Mat H_gripper2target_0 = H_base2gripper_0 * H_cam2base * H_target2cam_0;
    // ��ȡ�ο�����ת��ƽ��
    cv::Mat R_ref = H_gripper2target_0(cv::Rect(0, 0, 3, 3));
    cv::Mat t_ref = H_gripper2target_0(cv::Rect(3, 0, 1, 3));

    // ����3����������������������ο�ֵ��ƫ��
    for (int i = 0; i < N; i++) {
        // ������ǰ������ �������������궨������ ��ξ���
        cv::Mat H_base2gripper_i = composeHomogeneousMat(R_base2gripper[i], T_base2gripper[i]);
        cv::Mat H_target2cam_i = composeHomogeneousMat(R_target2cam[i], T_target2cam[i]);

        // ���㵱ǰ������ �������궨�� �任����
        //cv::Mat H_gripper2target_i = rigidMatInverse(H_base2gripper_i) * H_cam2base * H_target2cam_i;
        cv::Mat H_gripper2target_i = H_base2gripper_i * H_cam2base * H_target2cam_i;
        cv::Mat R_i = H_gripper2target_i(cv::Rect(0, 0, 3, 3));
        cv::Mat t_i = H_gripper2target_i(cv::Rect(3, 0, 1, 3));

        // ������ת��R_err = R_ref^T * R_i �� ��תƫ�����
        cv::Mat R_err = R_ref.t() * R_i;
        double rot_err = rotMat2Angle(R_err);
        rot_errors_deg.push_back(rot_err);

        // ����ƽ����t_i �� t_ref ��ŷ�Ͼ���
        double trans_err = calcTransError(t_ref, t_i);
        trans_errors_mm.push_back(trans_err);
    }
}

void calcHandEyeError_EyeInHand(
    const std::vector<cv::Mat>& R_gripper2base,
    const std::vector<cv::Mat>& T_gripper2base,
    const std::vector<cv::Mat>& R_target2cam,
    const std::vector<cv::Mat>& T_target2cam,
    const cv::Mat& R_cam2gripper,
    const cv::Mat& t_cam2gripper,
    std::vector<double>& rot_errors_deg,
    std::vector<double>& trans_errors_mm
) {
    rot_errors_deg.clear();
    trans_errors_mm.clear();
    int N = R_gripper2base.size();
    if (N < 2) {
        std::cerr << "Not enough data, need at least 2 samples!" << std::endl;
        return;
    }

    cv::Mat H_cam2gripper = composeHomogeneousMat(R_cam2gripper, t_cam2gripper);

    cv::Mat H_gripper2base_0 = composeHomogeneousMat(R_gripper2base[0], T_gripper2base[0]);
    cv::Mat H_target2cam_0 = composeHomogeneousMat(R_target2cam[0], T_target2cam[0]);
    cv::Mat H_target2base_0 = H_gripper2base_0 * H_cam2gripper * H_target2cam_0;
    
    cv::Mat R_ref = H_target2base_0(cv::Rect(0, 0, 3, 3));
    cv::Mat t_ref = H_target2base_0(cv::Rect(3, 0, 1, 3));

    for (int i = 0; i < N; i++) {
        cv::Mat H_gripper2base_i = composeHomogeneousMat(R_gripper2base[i], T_gripper2base[i]);
        cv::Mat H_target2cam_i = composeHomogeneousMat(R_target2cam[i], T_target2cam[i]);
        cv::Mat H_target2base_i = H_gripper2base_i * H_cam2gripper * H_target2cam_i;
        cv::Mat R_i = H_target2base_i(cv::Rect(0, 0, 3, 3));
        cv::Mat t_i = H_target2base_i(cv::Rect(3, 0, 1, 3));

        cv::Mat R_err = R_ref.t() * R_i;
        double rot_err = rotMat2Angle(R_err);
        rot_errors_deg.push_back(rot_err);

        double trans_err = calcTransError(t_ref, t_i);
        trans_errors_mm.push_back(trans_err);
    }
}

struct HandEyeTotalError {
    double rot_mean;   // ��ת����ֵ���㣩
    double rot_std;    // ��ת����׼��㣩
    double rot_max;    // ��ת������ֵ���㣩
    double trans_mean; // ƽ������ֵ��mm��
    double trans_std;  // ƽ������׼�mm��
    double trans_max;  // ƽ��������ֵ��mm��
};

// ͳ��������ƽ��ֵ����׼����ֵ��
HandEyeTotalError statTotalError(const std::vector<double>& rot_errors, const std::vector<double>& trans_errors) {
    HandEyeTotalError total_err;
    int N = rot_errors.size();

    // ������ת���ͳ��ֵ
    total_err.rot_mean = std::accumulate(rot_errors.begin(), rot_errors.end(), 0.0) / N;
    total_err.rot_max = *std::max_element(rot_errors.begin(), rot_errors.end());
    // �����׼��
    double rot_var = 0.0;
    for (double e : rot_errors) rot_var += std::pow(e - total_err.rot_mean, 2);
    total_err.rot_std = std::sqrt(rot_var / N);

    // ����ƽ�����ͳ��ֵ
    total_err.trans_mean = std::accumulate(trans_errors.begin(), trans_errors.end(), 0.0) / N;
    total_err.trans_max = *std::max_element(trans_errors.begin(), trans_errors.end());
    double trans_var = 0.0;
    for (double e : trans_errors) trans_var += std::pow(e - total_err.trans_mean, 2);
    total_err.trans_std = std::sqrt(trans_var / N);

    return total_err;
}

int cameraCalibration::get_error_data(){
    return error_data_;
}
pcl::PointCloud<pcl::PointXYZ> ext_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr ext_cloud_rest(new pcl::PointCloud<pcl::PointXYZ>);
bool segment_cloud_Material_box_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud, pcl::PointCloud<pcl::PointXYZ>& out_cloud, pcl::ModelCoefficients& out_coef) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud_cp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*sor_cloud, *sor_cloud_cp); // ע�⴫������ú�ĵ��ƶ���
    pcl::SACSegmentation<pcl::PointXYZ> sac;
    pcl::PointIndices::Ptr inliner(new  pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new  pcl::ModelCoefficients);
    sac.setInputCloud(sor_cloud);
    sac.setOptimizeCoefficients(true);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setModelType(pcl::SACMODEL_PLANE);
    sac.setMaxIterations(100);
    sac.setDistanceThreshold(30);

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

        //float ang = computeAngleEigen(PtsNormal, Eigen::Vector3d(0, 0, 1));
        //bool found_ang = fabs(ang) < 20 || fabs(ang - 180) < 20;
        out_coef = *coefficients;
        if (1.0 * ext_cloud.size() / sor_cloud_cp->points.size() > 0.6 ) {
            out_cloud = ext_cloud;
            break;
        }
    }
    return false;
}

bool ensureDirectoryExistsRecursive(std::string file_path) {
    if (access(file_path.c_str(), F_OK) != 0) {
        mode_t mode = 0755;
        std::string cmd = "mkdir -p " + std::string(file_path);
        int result = system(cmd.c_str());
        //mkdir(file_path.c_str(),mode);
    }
    return 1;
}

void cameraCalibration::write_image_data(cv::Mat color, cv::Mat dep,std::vector<float> intrinsic, float scale, cv::Mat result, pose6d pose) {
    if (access("save_calibrat_img_number.txt", 0) != 0) {
        LOG_INFO("can_not_find_save_calibrat_img_number.txt");
        cal_index_ = 0;
        save_count_ = 0;
    }
    else {
        if (app_first_) {
            app_first_ = 0;
            std::vector<std::string> read_lines;
            std::string s;
            std::ifstream inf;
            inf.open("save_calibrat_img_number.txt");
            if (inf.is_open()) {
                while (getline(inf, s)) {
                    read_lines.push_back(s);
                }
                inf.close();
                if (read_lines.size() > 0) {
                    cal_index_ = std::stod(read_lines[0]);
                    //save_count_ = std::stod(read_lines[1]);
                }
            }
        }
    }
std::string str_name = std::to_string(cal_index_) + "_" + std::to_string(save_count_);
    LOG_INFO("save_img_pose {}  {}", cal_index_, save_count_);
    save_count_++;
    std::string save_path = save_path_ + "/" + workspace_id_ + "/calibration";
    ensureDirectoryExistsRecursive(save_path);
    cv::imwrite(save_path + "/color_" + str_name + ".png", color);
    cv::imwrite(save_path + "/dep_" + str_name + ".png", dep);
    cv::imwrite(save_path + "/result" + str_name + ".png", result);
    std::string intrinsic_path = save_path + "/intrinsic_" + str_name + ".json";
    std::ofstream write_pose;
    write_pose.open(save_path + "/hand_pose_" + str_name + ".txt");
    if (write_pose.is_open()) {
        // 保存位置和四元数 (共7个值: x, y, z, qx, qy, qz, qw)
        write_pose << pose.x << std::endl;
        write_pose << pose.y << std::endl;
        write_pose << pose.z << std::endl;
        write_pose << pose.orientation_x << std::endl;
        write_pose << pose.orientation_y << std::endl;
        write_pose << pose.orientation_z << std::endl;
        write_pose << pose.orientation_w << std::endl;
    }

    // 保存内参和深度尺度到JSON文件
    nlohmann::json intrinsic_json;
    intrinsic_json["intrinsic"] = intrinsic;
    intrinsic_json["scale"] = scale;
    std::ofstream ofs(intrinsic_path);
    if (ofs.is_open()) {
        ofs << intrinsic_json.dump(4);
        ofs.close();
    }


    std::ofstream outf;
    outf.open("save_calibrat_img_number.txt");
    if (outf.is_open()) {
        outf << cal_index_ << std::endl;
        outf.close();
    }
}

cameraCalibration::cameraCalibration(){
    cal_index_ = 0;
    save_count_ = 0;
    error_data_ = 0;
    app_first_ = 1;
    total_rot_error_ = 0;
    mean_trans_error_ = 0;
    save_path_ = "save_cal_data";
    workspace_id_ = "default";
    //detector_mark_.setMarkType(MY_TEST); //TAG16h5  TAG36h11 ARUCO_MIP_16h3
    //mark3d_.read_param_json();
}

cameraCalibration::~cameraCalibration(){
}

void cameraCalibration::get_cam_img(pose6d& pose){
    cv::Mat color;
    cv::Mat dep;
    cv::Mat xyz;
    cv::Mat color_cp;
    int error_data = 0;
    error_data_ = error_data;
    if (0 != error_data_) {
        LOG_INFO("get_camera_img_error!!!  {}", error_data);
        return;
    }

    if (!color.empty() && !dep.empty() && !xyz.empty()) {
        //std::thread{ std::bind(&cameraCalibration::write_image_data, this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4),color,dep,xyz,pose }.detach();
        find_apriltag(color, xyz, pose);
    }

}

cv::Mat poseVectorToTransformMatrix(const pose_euler_angle& pose)
{
    // 1. 获取平移和旋转向量
    double tx = static_cast<double>(pose.x);
    double ty = static_cast<double>(pose.y);
    double tz = static_cast<double>(pose.z);
    double rx = static_cast<double>(pose.theta_x);
    double ry = static_cast<double>(pose.theta_y);
    double rz = static_cast<double>(pose.theta_z);

    // 2. 计算各轴旋转矩阵
    // 绕X轴旋转矩阵
    cv::Mat Rx = (cv::Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos(rx), -sin(rx),
        0, sin(rx), cos(rx));

    // 绕Y轴旋转矩阵
    cv::Mat Ry = (cv::Mat_<double>(3, 3) <<
        cos(ry), 0, sin(ry),
        0, 1, 0,
        -sin(ry), 0, cos(ry));

    // 绕Z轴旋转矩阵
    cv::Mat Rz = (cv::Mat_<double>(3, 3) <<
        cos(rz), -sin(rz), 0,
        sin(rz), cos(rz), 0,
        0, 0, 1);

    // 3. 计算旋转矩阵XYZ顺序
    cv::Mat R;
    switch (pose.euler_rot_order_type) {
    case 1:
        R = Rz * Ry * Rx;
        break;
    case 2:
        R = Ry * Rz * Ry;
        break;
    case 3:
        R = Rz * Rx * Ry;
        break;
    case 4:
        R = Rx * Rz * Ry;
        break;
    case 5:
        R = Ry * Rx * Rz;
        break;
    case 6:
        R = Rx * Ry * Rz;
        break;
    default:
        R = Rz * Ry * Rx;
        break;
    }

    // 4. 构建4x4变换矩阵
    cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);

    // 将旋转矩阵复制到左上角3x3区域
    R.copyTo(transform(cv::Rect(0, 0, 3, 3)));

    // 设置平移向量
    transform.at<double>(0, 3) = tx;
    transform.at<double>(1, 3) = ty;
    transform.at<double>(2, 3) = tz;

    return transform;
}


void cameraCalibration::get_cam_img(camera_callback_data& cam_data,pose6d& hand_pose){
    LOG_INFO("get_camera_data_error !!!! {}",cam_data.error_data);
    error_data_ = cam_data.error_data;
    if (!cam_data.color.empty() && !cam_data.dep.empty() && cam_data.intrinsic.size() == 9 && cam_data.error_data ==0) {
        cv::resize(cam_data.color, cam_data.color, cam_data.dep.size());
        cv::Mat color = cam_data.color.clone();
        cv::Mat dep = cam_data.dep.clone();
        std::vector<float> intrinsic = cam_data.intrinsic;
        float scale = cam_data.scale;
        cv::Mat xyz = cv::Mat(dep.size(), CV_32FC3);//CV_16UC3
        ushort* depdata = (ushort*)dep.data;
        float* xyzdata = (float*)xyz.data;
        float dep_z;
        float dep_x;
        float dep_y;
        for (int y = 0; y < dep.rows; ++y)
            for (int x = 0; x < dep.cols; ++x) {
                dep_z = scale * depdata[y * dep.cols + x];
                dep_x = (x - intrinsic[2]) * dep_z / intrinsic[0];
                dep_y = (y - intrinsic[5]) * dep_z / intrinsic[4];
                xyzdata[3 * y * xyz.cols + 3 * x] = dep_x;
                xyzdata[3 * y * xyz.cols + 3 * x + 1] =dep_y;
                xyzdata[3 * y * xyz.cols + 3 * x + 2] = dep_z;

            }
        cv::Mat pose = pose6dToMat44(hand_pose);
        std::vector<Marker> marks;
        find_apriltag(color, xyz, pose,marks);
        for (int i = 0; i < marks.size(); ++i) {
            for (int j = 0; j < marks[i].pt.size() - 1; ++j)
                cv::line(color, marks[i].pt[j], marks[i].pt[j + 1], cv::Scalar(0, 0, 255));
        }
        std::thread{ std::bind(&cameraCalibration::write_image_data, this,
                               std::placeholders::_1,
                               std::placeholders::_2,
                               std::placeholders::_3,
                               std::placeholders::_4,
                               std::placeholders::_5,
                               std::placeholders::_6),cam_data.color,cam_data.dep,intrinsic,scale,color,hand_pose }.detach();

    }else
        error_data_ = -10;
}

// ������������
double distanceBetweenPoints(const cv::Point3f& p1, const cv::Point3f& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    //return std::sqrt(dx * dx + dy * dy + dz * dz);
    return dx * dx + dy * dy + dz * dz;
}

// ����㵽ƽ��ľ��루ƽ�淽�̣�ax + by + cz + d = 0��
double pointToPlaneDistance(const cv::Point3f& p, double a, double b, double c, double d) {
    return std::fabs(a * p.x + b * p.y + c * p.z + d) / std::sqrt(a * a + b * b + c * c);
}

// ��̬����RANSAC�����������
// confidence�����Ŷȣ���0.99��ʾ99%�����ҵ���ȷģ�ͣ�
// outlierRatio��Ԥ����Ⱥ���������0.5��ʾ50%��Ⱥ�㣩
int calculateMaxIterations(double confidence, double outlierRatio) {
    if (outlierRatio >= 1.0) return 1000; // �����������
    double p = 1.0 - std::pow(1.0 - outlierRatio, 3); // һ�β���ѡ�����ڵ�ĸ���
    return static_cast<int>(std::ceil(std::log(1.0 - confidence) / std::log(1.0 - p)));
}

// RANSACƽ����ϣ��������������Ż���
// ������
//   points: ������ά�㼯
//   inliers: ����ڵ�����
//   threshold: ������ֵ���ж��ڵ㣩
//   minDist: ��������С��ࣨ���˽�����㣩
//   confidence: ���Ŷȣ�0.0~1.0��
// ���أ�ƽ����� (a, b, c, d)
bool ransacPlaneFitting(const std::vector<cv::Point3f>& points, cv::Vec4d& bestPlane,
    double threshold = 0.01, double minDist = 0.0001,//0.005,
    double confidence = 0.99) {
    if (points.size() < 3) {
        return false; // �㼯����3�����޷����
    }
    std::vector<int> inliers;
    int point_size = points.size();

    // Ԥ����Ⱥ���������ʼ��Ϊ0.5����̬������
    double outlierRatio = 0.5;
    int maxIterations = calculateMaxIterations(confidence, outlierRatio);
    int bestInlierCount = 0;
    bestPlane = cv::Vec4d(0, 0, 0, 0);

    // �����������
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, static_cast<int>(points.size() - 1));

    for (int iter = 0; iter < maxIterations; ++iter) {
        // 1. ���ѡ��3�����ظ��Ҿ���ϸ�ĵ�
        int idx1, idx2, idx3;
        bool validSample = false;
        int retryCount = 0;
        const int maxRetry = 15; // ������Դ�����������ѭ��

        while (!validSample && retryCount < maxRetry) {
            idx1 = dis(gen);
            do { idx2 = dis(gen); } while (idx2 == idx1);
            do { idx3 = dis(gen); } while (idx3 == idx1 || idx3 == idx2);

            // �������Ƿ����Ҫ��
            double d12 = distanceBetweenPoints(points[idx1], points[idx2]);
            double d13 = distanceBetweenPoints(points[idx1], points[idx3]);
            if (d12 >= minDist && d13 >= minDist) {
                validSample = true;
            }
            else {
                retryCount++;
            }
        }

        // ����β���ʧ�ܣ�������ǰ������������Ч���㣩
        if (!validSample) continue;

        // 2. ����ƽ��ģ��
        const cv::Point3f& p1 = points[idx1];
        const cv::Point3f& p2 = points[idx2];
        const cv::Point3f& p3 = points[idx3];

        // �������������
        cv::Point3f v1 = p2 - p1;
        cv::Point3f v2 = p3 - p1;
        cv::Point3f normal = v1.cross(v2);

        // �������ӽ��㣨���㹲�ߣ�������
        if (normal.dot(normal) < 1e-10) continue;

        // ƽ�����
        double a = normal.x;
        double b = normal.y;
        double c = normal.z;
        double d = -(a * p1.x + b * p1.y + c * p1.z);

        // 3. ͳ���ڵ�
        std::vector<int> currentInliers;
        currentInliers.reserve(points.size());
        for (int i = 0; i < points.size(); ++i) {
            double dist = pointToPlaneDistance(points[i], a, b, c, d);
            if (dist < threshold) {
                currentInliers.push_back(i);
            }
        }

        // 4. ��������ģ�ͣ�����̬������������
        if (currentInliers.size() > bestInlierCount) {
            bestInlierCount = currentInliers.size();
            bestPlane = cv::Vec4d(a, b, c, d);
            inliers = currentInliers;

            // ����Ԥ����Ⱥ����������ٺ�����������
            outlierRatio = 1.0 - static_cast<double>(bestInlierCount) / points.size();
            maxIterations = std::min(maxIterations, calculateMaxIterations(confidence, outlierRatio));

            if (inliers.size() > point_size * 0.6)
                break;
        }
    }

    if (inliers.size() <= point_size * 0.4)
        return false;

    // 5. �������ڵ��������ƽ�棨��߾��ȣ�
    if (inliers.size() >= 3) {
        // �����ڵ����ĵ�
        cv::Point3f centroid(0, 0, 0);
        for (int idx : inliers) {
            centroid.x += points[idx].x;
            centroid.y += points[idx].y;
            centroid.z += points[idx].z;
        }
        centroid.x /= inliers.size();
        centroid.y /= inliers.size();
        centroid.z /= inliers.size();

        // ����Э�������
        cv::Mat cov(3, 3, CV_64F, cv::Scalar(0));
        for (int idx : inliers) {
            cv::Point3f p = points[idx] - centroid;
            cov.at<double>(0, 0) += p.x * p.x;
            cov.at<double>(0, 1) += p.x * p.y;
            cov.at<double>(0, 2) += p.x * p.z;
            cov.at<double>(1, 1) += p.y * p.y;
            cov.at<double>(1, 2) += p.y * p.z;
            cov.at<double>(2, 2) += p.z * p.z;
        }
        cov /= (inliers.size() - 1);

        // SVD�ֽ������ŷ���������С����ֵ��Ӧ��������
        cv::Mat w, u, vt;
        cv::SVD::compute(cov, w, u, vt);
        cv::Vec3d normal(u.at<double>(0, 2), u.at<double>(1, 2), u.at<double>(2, 2));
        double a = normal[0];
        double b = normal[1];
        double c = normal[2];
        double d = -(a * centroid.x + b * centroid.y + c * centroid.z);
        bestPlane = cv::Vec4d(a, b, c, d);
    }

    return true;
}

void get_camera_point3d(std::vector<cv::Point2f> corners, cv::Mat H, cv::Vec4d PtsNormal, std::vector<cv::Point3f>& camera_point3d)
{
    double h[9];
    h[0] = H.at<double>(0, 0);
    h[1] = H.at<double>(0, 1);
    h[2] = H.at<double>(0, 2);
    h[3] = H.at<double>(1, 0);
    h[4] = H.at<double>(1, 1);
    h[5] = H.at<double>(1, 2);
    h[6] = H.at<double>(2, 0);
    h[7] = H.at<double>(2, 1);
    h[8] = H.at<double>(2, 2);

    cv::Point3d temp;
    double ff = 0;
    for (int i = 0; i < corners.size(); ++i)
    {
        ff = h[6] * corners[i].x + h[7] * corners[i].y + h[8];
        temp.x = (h[0] * corners[i].x + h[1] * corners[i].y + h[2]) / ff;
        temp.y = (h[3] * corners[i].x + h[4] * corners[i].y + h[5]) / ff;
        temp.z = (-PtsNormal[0] * temp.x - PtsNormal[1] * temp.y - PtsNormal[3]) / PtsNormal[2];

        camera_point3d.push_back(temp);
    }
}

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
        Matrix3d eMat;
        eMat(0, 0) = meanXX - meanX * meanX; eMat(0, 1) = meanXY - meanX * meanY; eMat(0, 2) = meanXZ - meanX * meanZ;
        eMat(1, 0) = meanXY - meanX * meanY; eMat(1, 1) = meanYY - meanY * meanY; eMat(1, 2) = meanYZ - meanY * meanZ;
        eMat(2, 0) = meanXZ - meanX * meanZ; eMat(2, 1) = meanYZ - meanY * meanZ; eMat(2, 2) = meanZZ - meanZ * meanZ;
        Eigen::EigenSolver<Eigen::Matrix3d> xjMat(eMat);
        Matrix3d eValue = xjMat.pseudoEigenvalueMatrix();
        Matrix3d eVector = xjMat.pseudoEigenvectors();
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
void cameraCalibration::get_mark_positon(std::vector<Marker>& marks, cv::Mat& xyz,std::vector<mark3dPosition>& mark3ds) {
    float* xyzdata = (float*)xyz.data;
    std::vector<cv::Point2f> corners;
    int minx = 100000;
    int maxx = 0;
    int miny = 100000;
    int maxy = 0;
    for (int i = 0; i < marks.size(); ++i) {
        for (int j = 0; j < marks[i].pt.size(); ++j){
            if (minx > marks[i].pt[j].x)
                minx = marks[i].pt[j].x;
            if (miny > marks[i].pt[j].y)
                miny = marks[i].pt[j].y;
            if (maxx < marks[i].pt[j].x)
                maxx = marks[i].pt[j].x;
            if (maxy < marks[i].pt[j].y)
                maxy = marks[i].pt[j].y;
            corners.push_back(marks[i].pt[j]);
        }
    }
    float tp_z = 0;
    cv::Point3f pt;
    std::vector<cv::Point2f> imgpt;
    std::vector<cv::Point2f> cloud_point_xy;
    std::vector<cv::Point3f> cloud_xyz;

    pcl::PointCloud<pcl::PointXYZ> cloud_xyz_pcl;
    for (int y = miny; y < maxy; ++y) {
        for (int x = minx; x < maxx; ++x) {
            tp_z = xyzdata[3 * y * xyz.cols + 3 * x + 2];
            if (tp_z >  2000|| tp_z < 100)
                continue;
            pt.x = (xyzdata[3 * y * xyz.cols + 3 * x] );
            pt.y = (xyzdata[3 * y * xyz.cols + 3 * x + 1]);
            pt.z = (xyzdata[3 * y * xyz.cols + 3 * x + 2]);
            imgpt.push_back(cv::Point2f(x, y));
            cloud_point_xy.push_back(cv::Point2f(pt.x, pt.y));
            cloud_xyz.push_back(pt);
            cloud_xyz_pcl.emplace_back(pt.x, pt.y, pt.z);
        }
    }
    cv::Vec4d PtsNormal;
    ransacPlaneFitting(cloud_xyz, PtsNormal, 0.01, 0.0001, 0.99);

    pcl::PointCloud<pcl::PointXYZ> out_cloud;
    pcl::ModelCoefficients out_coef;
    segment_cloud_Material_box_plane(cloud_xyz_pcl.makeShared(), out_cloud, out_coef);
    std::vector<double> pParam;
    fit_plane_by_least_squares(pParam, out_cloud.makeShared());

    double noram_tp = sqrt(PtsNormal[0] * PtsNormal[0] + PtsNormal[1] * PtsNormal[1] + PtsNormal[2] * PtsNormal[2]);
    PtsNormal[0] = PtsNormal[0] / noram_tp;
    PtsNormal[1] = PtsNormal[1] / noram_tp;
    PtsNormal[2] = PtsNormal[2] / noram_tp;
    PtsNormal[3] = PtsNormal[3] / noram_tp;

    PtsNormal[0] = pParam[0];
    PtsNormal[1] = pParam[1];
    PtsNormal[2] = pParam[2];
    PtsNormal[3] = pParam[3];


    double dist = 0;
    double max_dist = 0;
    std::vector<double> dist_ve;
    for (int i = 0; i < cloud_xyz.size(); ++i) {
        dist = PtsNormal[0] * cloud_xyz[i].x + PtsNormal[1] * cloud_xyz[i].y + PtsNormal[2] * cloud_xyz[i].z + PtsNormal[3];
        if (fabs(dist) > max_dist)
            max_dist = fabs(dist);
        dist_ve.push_back(dist);
    }


    cv::Mat H = cv::findHomography(imgpt, cloud_point_xy, cv::RANSAC);

    std::vector<cv::Point3f> camera_point3d;
    get_camera_point3d(corners, H, PtsNormal, camera_point3d);

    mark3dPosition tp3d;
    int index = 0;
    for (int i = 0; i < marks.size(); ++i) {
        //std::cout << i << "  " << "    " << index << std::endl;
        tp3d.p1 = camera_point3d[index++];
        tp3d.p2 = camera_point3d[index++];
        tp3d.p3 = camera_point3d[index++];
        tp3d.p4 = camera_point3d[index++];
        tp3d.id = marks[i].id;
        mark3ds.push_back(tp3d);
    }
}
cv::Mat pose6dToMat44(const pose6d& pose) {
    // 1. ��ʼ��4x4���󣨵�λ����
    cv::Mat mat = cv::Mat::eye(4, 4, CV_64FC1);
    // 2. ��ȡƽ�Ʒ�����ת��Ϊdouble���ͣ�
    mat.at<double>(0, 3) = static_cast<double>(pose.x);
    mat.at<double>(1, 3) = static_cast<double>(pose.y);
    mat.at<double>(2, 3) = static_cast<double>(pose.z);

    // 3. ��ȡ��Ԫ����x, y, z, w��������һ��
    Eigen::Quaterniond q(
        static_cast<double>(pose.orientation_w),  // w��ʵ����
        static_cast<double>(pose.orientation_x),  // x���鲿��
        static_cast<double>(pose.orientation_y),  // y���鲿��
        static_cast<double>(pose.orientation_z)   // z���鲿��
    );
    q.normalize();  // ȷ����Ԫ���ǵ�λ��Ԫ��

    // 4. ��Ԫ��ת��ת��������Ǳ�ʾ���Ƕȵ�λΪ���ȣ�
    Eigen::AngleAxisd angle_axis(q);
    cv::Mat rot_vec(3, 1, CV_64FC1);
    rot_vec.at<double>(0) = angle_axis.axis()[0] * angle_axis.angle();
    rot_vec.at<double>(1) = angle_axis.axis()[1] * angle_axis.angle();
    rot_vec.at<double>(2) = angle_axis.axis()[2] * angle_axis.angle();

    // 5. ��ת����ת3x3��ת����OpenCV��Rodrigues������
    cv::Mat rot_mat;
    cv::Rodrigues(rot_vec, rot_mat);  // rot_matΪ3x3 CV_64FC1����

    // 6. ����ת����д��4x4��������Ͻ�3x3����
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mat.at<double>(i, j) = rot_mat.at<double>(i, j);
        }
    }
    return mat;
}

Vector3d computeCentroid2(const std::vector<cv::Point3f>& points) {
    Vector3d centroid(0, 0, 0);
    for (const auto& p : points) {
        centroid.x() += p.x;
        centroid.y() += p.y;
        centroid.z() += p.z;
    }
    centroid /= points.size();
    return centroid;
}

cv::Mat computeRigidTransform(const std::vector<cv::Point3f>& P, const std::vector<cv::Point3f>& Q) {
    if (P.size() != Q.size() || P.empty()) {
        LOG_INFO("makr_ponit!=camera_point_or_point_empty");
        return cv::Mat();
    }

    // ��������
    Vector3d p_centroid = computeCentroid2(P);
    Vector3d q_centroid = computeCentroid2(Q);

    // ����Э�������H�������﷨����
    Matrix3d H = Matrix3d::Zero();
    for (size_t i = 0; i < P.size(); ++i) {
        // �������ȴ����������ټ�ȥ���ģ����Ӹ�ֵ������
        Vector3d p = Vector3d(P[i].x, P[i].y, P[i].z) - p_centroid;
        Vector3d q = Vector3d(Q[i].x, Q[i].y, Q[i].z) - q_centroid;
        H += p * q.transpose();
    }

    // SVD�ֽ�
    JacobiSVD<Matrix3d> svd(H, ComputeFullU | ComputeFullV);
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();

    // ������ת����R����������
    Matrix3d R = V * U.transpose();
    if (R.determinant() < 0) {
        Matrix3d diag = Matrix3d::Identity();
        diag(2, 2) = -1;
        R = V * diag * U.transpose();
    }

    // ����ƽ������t
    Vector3d t = q_centroid - R * p_centroid;
    /////////////////////////////////////////////////////////
    cv::Mat T = cv::Mat::eye(4, 4, CV_64FC1);

    // ������ת����ǰ3x3���֣�
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            T.at<double>(i, j) = R(i, j);
        }
    }

    // ����ƽ����������4��ǰ3�У�
    T.at<double>(0, 3) = t.x();  // tx
    T.at<double>(1, 3) = t.y();  // ty
    T.at<double>(2, 3) = t.z();  // tz

    return T;
}

void cameraCalibration::find_apriltag(cv::Mat& color,cv::Mat& xyz,pose6d& pose){
    std::vector<Marker> marks;
    detector_mark_.detect(color, marks);
    if(marks.size()>4){
        std::vector<cv::Point3f> cam_marks3dpoint;
        std::vector<cv::Point3f> marks3dpoint;
        pose_.push_back(pose);
        cv::Mat pose_M = pose6dToMat44(pose);
        std::vector<mark3dPosition> mark3ds;
        get_mark_positon(marks, xyz, mark3ds);
        for (int i = 0; i < mark3ds.size(); ++i) {
            mark3dPosition tmp;
            bool is_found = mark3d_.get_mark3d_position(mark3ds[i].id, tmp);
            if (is_found) {
                marks3dpoint.push_back(tmp.p1);
                marks3dpoint.push_back(tmp.p2);
                marks3dpoint.push_back(tmp.p3);
                marks3dpoint.push_back(tmp.p4);

                cam_marks3dpoint.push_back(mark3ds[i].p1);
                cam_marks3dpoint.push_back(mark3ds[i].p2);
                cam_marks3dpoint.push_back(mark3ds[i].p3);
                cam_marks3dpoint.push_back(mark3ds[i].p4);
            }
        }
        cv::Mat Tcm = computeRigidTransform(marks3dpoint, cam_marks3dpoint);
        math_pt3d_.hand_pose.push_back(pose_M);
    }
    else {
        LOG_INFO("Marker_size<=4");
    }
}

void cameraCalibration::find_apriltag(cv::Mat& color, cv::Mat& xyz, cv::Mat& pose,std::vector<Marker>& marks) {
    detector_mark_.detect(color, marks);
    if (marks.size() > 4) {
        std::vector<cv::Point3f> cam_marks3dpoint;
        std::vector<cv::Point3f> marks3dpoint;

        pcl::PointCloud<pcl::PointXYZ> in_could;
        pcl::PointCloud<pcl::PointXYZ> out_could;
        //pose_.push_back(pose);
        //cv::Mat pose_M = pose6dToMat44(pose);
        std::vector<mark3dPosition> mark3ds;
        get_mark_positon(marks, xyz, mark3ds);
        for (int i = 0; i < mark3ds.size(); ++i) {
            mark3dPosition tmp;
            bool is_found = mark3d_.get_mark3d_position(mark3ds[i].id, tmp);
            if (is_found) {
                marks3dpoint.push_back(tmp.p1);
                marks3dpoint.push_back(tmp.p2);
                marks3dpoint.push_back(tmp.p3);
                marks3dpoint.push_back(tmp.p4);

                cam_marks3dpoint.push_back(mark3ds[i].p1);
                cam_marks3dpoint.push_back(mark3ds[i].p2);
                cam_marks3dpoint.push_back(mark3ds[i].p3);
                cam_marks3dpoint.push_back(mark3ds[i].p4);
                ///////////////////////////////////
                in_could.emplace_back(tmp.p1.x, tmp.p1.y, tmp.p1.z);
                in_could.emplace_back(tmp.p2.x, tmp.p2.y, tmp.p2.z);
                in_could.emplace_back(tmp.p3.x, tmp.p3.y, tmp.p3.z);
                in_could.emplace_back(tmp.p4.x, tmp.p4.y, tmp.p4.z);

                out_could.emplace_back(mark3ds[i].p1.x, mark3ds[i].p1.y, mark3ds[i].p1.z);
                out_could.emplace_back(mark3ds[i].p2.x, mark3ds[i].p2.y, mark3ds[i].p2.z);
                out_could.emplace_back(mark3ds[i].p3.x, mark3ds[i].p3.y, mark3ds[i].p3.z);
                out_could.emplace_back(mark3ds[i].p4.x, mark3ds[i].p4.y, mark3ds[i].p4.z);
            }
        }
        cv::Mat Tcm = computeRigidTransform(marks3dpoint, cam_marks3dpoint);
        //cv::Mat Tcm = computeRigidTransform(cam_marks3dpoint, marks3dpoint);
        if(!Tcm.empty()){
            math_pt3d_.hand_pose.push_back(pose);
            math_pt3d_.Tmark2cam.push_back(Tcm);
            error_data_ = 0;
        }else{
            LOG_INFO("computeRigidTransform failed, Tcm is empty!");
            error_data_ = -106; // Define a new error code for transformation calculation failure
        }
        /*
        /////////////////////////////
        for (int i = 0; i < cam_marks3dpoint.size(); ++i) {
            std::cout << cam_marks3dpoint[i].x << "," << cam_marks3dpoint[i].y << "," << cam_marks3dpoint[i].z << " | ";
        }
        std::cout << std::endl;
        for (int i = 0; i < marks3dpoint.size(); ++i) {
            std::cout << marks3dpoint[i].x << "," << marks3dpoint[i].y << "," << marks3dpoint[i].z << " | ";
        }
        std::cout << std::endl;
        std::cout <<" -----------------------"<< std::endl;
        //////////
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation;
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
        TESVD.estimateRigidTransformation(in_could, out_could, transformation);

        cv::Mat Tcm_clone = cv::Mat::eye(4, 4, CV_64FC1);
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                Tcm_clone.at<double>(i,j) = double(transformation(i, j)) ;

        double error_d = 0;
        for (int i = 0; i < marks3dpoint.size(); ++i) {
            double x = marks3dpoint[i].x* Tcm.at<double>(0, 0) + marks3dpoint[i].y * Tcm.at<double>(0, 1) + marks3dpoint[i].z * Tcm.at<double>(0, 2) + Tcm.at<double>(0, 3);
            double y = marks3dpoint[i].x * Tcm.at<double>(1, 0) + marks3dpoint[i].y * Tcm.at<double>(1, 1) + marks3dpoint[i].z * Tcm.at<double>(1, 2) + Tcm.at<double>(1, 3);
            double z = marks3dpoint[i].x * Tcm.at<double>(2, 0) + marks3dpoint[i].y * Tcm.at<double>(2, 1) + marks3dpoint[i].z * Tcm.at<double>(2, 2) + Tcm.at<double>(2, 3);
            double dxyz = sqrt((x - cam_marks3dpoint[i].x) * (x - cam_marks3dpoint[i].x) + (y - cam_marks3dpoint[i].y) * (y - cam_marks3dpoint[i].y) + (z - cam_marks3dpoint[i].z) * (z - cam_marks3dpoint[i].z));
            error_d = error_d + dxyz;
        }
        //std::cout << error_d << std::endl;
        //*/

    }
    else {
        error_data_ = -101;
        LOG_INFO("Marker_size<=4");
    }
}
void cameraCalibration::cam_calibration(std::string file_name,float& total_rot_error,float& mean_trans_error,std::vector<float>& cam_rt) {
    save_json_file_ = file_name;
    cal_index_++;
    save_count_ = 0;
    if (cal_index_ > 20)
        cal_index_ = 0;
    std::ofstream outf;
    outf.open("save_calibrat_img_number.txt");
    if (outf.is_open()) {
        outf << cal_index_ << std::endl;
        outf.close();
    }

    if (math_pt3d_.hand_pose.size() < 3) {
        LOG_INFO("hand_pose.size()<3!!!  ");
        error_data_ = -102;
        math_pt3d_.clear();
        return;
    }
    LOG_INFO("hand_pose.size  !!!  {}", math_pt3d_.hand_pose.size());
    std::vector<cv::Mat> R_base2gripper, T_base2gripper, R_target2cam, T_target2cam, R_gripper2base, T_gripper2base;
    for (int i = 0; i < math_pt3d_.hand_pose.size(); ++i) {
        if(math_pt3d_.Tmark2cam[i].empty() || math_pt3d_.hand_pose[i].empty())
        {
             LOG_INFO("Skip invalid data at index {}", i);
             continue;
        }

        cv::Mat tm1_inv = math_pt3d_.hand_pose[i].inv();
        cv::Mat Tbh_R = tm1_inv(cv::Rect(0, 0, 3, 3)).clone();
        cv::Mat Tbh_t = tm1_inv(cv::Rect(3, 0, 1, 3)).clone();
        R_base2gripper.push_back(Tbh_R);
        T_base2gripper.push_back(Tbh_t);
        /////////////////////////////////
        cv::Mat Thb_R = math_pt3d_.hand_pose[i](cv::Rect(0, 0, 3, 3)).clone();
        cv::Mat Thb_t = math_pt3d_.hand_pose[i](cv::Rect(3, 0, 1, 3)).clone();
        R_gripper2base.push_back(Thb_R);
        T_gripper2base.push_back(Thb_t);


        cv::Mat Tmc = math_pt3d_.Tmark2cam[i](cv::Rect(0, 0, 3, 3)).clone();
        cv::Mat Tmc_t = math_pt3d_.Tmark2cam[i](cv::Rect(3, 0, 1, 3)).clone();
        R_target2cam.push_back(Tmc);
        T_target2cam.push_back(Tmc_t);
    }
    cv::Mat R_cam2base, t_cam2base;
    // 根据hand_eye_type选择标定方法
    if (hand_eye_type_ == 1) {
        // 眼在手外 (Eye-to-Hand)
        calibrateHandEye(R_base2gripper, T_base2gripper, R_target2cam, T_target2cam, R_cam2base, t_cam2base, cv::CALIB_HAND_EYE_PARK);
        LOG_INFO("Using Eye-to-Hand calibration method");
    } else if (hand_eye_type_ == 2) {
        // 眼在手上 (Eye-in-Hand)
        calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam, T_target2cam, R_cam2base, t_cam2base, cv::CALIB_HAND_EYE_TSAI);
        LOG_INFO("Using Eye-in-Hand calibration method");
    } else {
        // 默认使用眼在手外
        calibrateHandEye(R_base2gripper, T_base2gripper, R_target2cam, T_target2cam, R_cam2base, t_cam2base, cv::CALIB_HAND_EYE_PARK);
        LOG_WARN("Unknown hand_eye_type: {}, using Eye-to-Hand", hand_eye_type_);
    }
    if (!R_cam2base.empty()&& !t_cam2base.empty()) {
        std::vector<double> rot_errors, trans_errors;
        if (hand_eye_type_ == 1) {
            calcHandEyeError_EyeToHand(R_base2gripper, T_base2gripper,
                R_target2cam, T_target2cam,
                R_cam2base, t_cam2base,
                rot_errors, trans_errors);
        } else if (hand_eye_type_ == 2) {
            calcHandEyeError_EyeInHand(R_gripper2base, T_gripper2base,
                R_target2cam, T_target2cam,
                R_cam2base, t_cam2base,
                rot_errors, trans_errors);
        } else {
            calcHandEyeError_EyeToHand(R_base2gripper, T_base2gripper,
                R_target2cam, T_target2cam,
                R_cam2base, t_cam2base,
                rot_errors, trans_errors);
        }
        // 2. Statistic total error
        HandEyeTotalError total_err = statTotalError(rot_errors, trans_errors);
        if(total_err.rot_mean>10||total_err.trans_mean>100){
            error_data_ = -105;
            LOG_INFO("calibration_result_error!  {}  {}",total_err.rot_mean,total_err.trans_mean);
        }
        total_rot_error = total_err.rot_mean;
        mean_trans_error = total_err.trans_mean;

        std::vector<std::vector<double>> camera_rt;
        camera_rt.resize(4);
        for (int i = 0; i < 3; ++i){
            for (int j = 0; j < 3; ++j){
                camera_rt[i].push_back(R_cam2base.at<double>(i, j));
            }
            camera_rt[i].push_back(t_cam2base.at<double>(i));
        }
        camera_rt[3].push_back(0);
        camera_rt[3].push_back(0);
        camera_rt[3].push_back(0);
        camera_rt[3].push_back(1);
        for(int i = 0;i < camera_rt.size();++i)
            for(int j = 0;j < camera_rt[i].size();++j)
                cam_rt.push_back(camera_rt[i][j]);

        std::string cam_config_abs_path = save_json_file_;//get_cam_config_path(save_json_file_);

        nlohmann::json cam_config;
        if (!load_camera_config(cam_config_abs_path, cam_config)) {
            error_data_ = -103;
            return;
        }
        
        // 确保 workspaces 数组存在
        if (!cam_config.contains("workspaces") || !cam_config["workspaces"].is_array()) {
            cam_config["workspaces"] = nlohmann::json::array();
        }
        
        bool workspace_found = false;
        for (auto& workspace : cam_config["workspaces"]) {
            if (workspace.contains("workspace_id") && 
                workspace["workspace_id"].get<std::string>() == workspace_id_) {
                workspace["hand_eye_matrix"] = camera_rt;
                workspace_found = true;
                break;
            }
        }
        
        // 如果没有找到对应的工作空间，创建一个新的
        if (!workspace_found) {
            nlohmann::json new_workspace;
            new_workspace["workspace_id"] = workspace_id_;
            new_workspace["hand_eye_matrix"] = camera_rt;
            cam_config["workspaces"].push_back(new_workspace);
        }
        
        try {
            std::ofstream output_file(cam_config_abs_path);
            if (!output_file.is_open()) {
                error_data_ = -104;
                std::cout << "save_opencv_file_fail: " << cam_config_abs_path << std::endl;
                return;
            }
            output_file << cam_config.dump(4);
            output_file.close();
        } catch (const std::exception& e) {
            error_data_ = -104;
            std::cerr << "Error saving camera config: " << e.what() << std::endl;
        }
        /*
        try {
            std::ofstream output_file(cam_config_abs_path);
            if (!output_file.is_open()) {
                std::cout << "save_opencv_file_fail: "<<cam_config_abs_path << std::endl;
                return;
            }
            //output_file << cam_config.dump(4);
            output_file << json_dumps_with_tab(cam_config) << std::endl;
            output_file.close();
            std::cout << "save_json_success!" << std::endl;
        } catch (const std::exception& e) {
            error_data_ = -104;
            std::cout << "save_fail: " << e.what() << std::endl;
            return;
        }*/
        error_data_ = 0;
    }

    math_pt3d_.clear();
}

void cameraCalibration::cancel() {
    LOG_INFO("cancel_calibration!!!  ");
    cal_index_++;
    save_count_ = 0;
    if (cal_index_ > 20)
        cal_index_ = 0;
    math_pt3d_.clear();
}
void cameraCalibration::cancel_last_detect(){
    if(math_pt3d_.hand_pose.size()>0&&math_pt3d_.Tmark2cam.size()>0){
        math_pt3d_.hand_pose.pop_back();
        math_pt3d_.Tmark2cam.pop_back();
    }
}

namespace fs = std::filesystem;
std::string cameraCalibration::get_cam_config_path(std::string file_name) {
    const std::string code_file_path = __FILE__;
    const fs::path code_dir = fs::path(code_file_path).parent_path();
    const fs::path ultirobotics_root = code_dir / "../..";
    const fs::path cam_config_relative = "ultirobotics_setups/config/vision/" + file_name + ".json";
    const fs::path cam_config_abs_path = fs::absolute(ultirobotics_root / cam_config_relative);

    std::cout<<"Code file directory:  "<<code_dir.c_str()<<std::endl;
    std::cout<<"Calculated cam config path:  "<<cam_config_abs_path.c_str()<<std::endl;

    return cam_config_abs_path.string();
}

bool cameraCalibration::load_camera_config(const std::string& config_abs_path, nlohmann::json& config) {
    if (access(config_abs_path.c_str(), 0) != 0) {
        config = nlohmann::json::object();
        LOG_INFO("can_not_find_param_save_image_number.txt");
        return true;
    }else
    {
        std::ifstream config_file(config_abs_path);
        if (!config_file.is_open()) {
            std::cout<<"Failed to open camera config file! Path: "<<config_abs_path.c_str()<<std::endl;
            return false;
        }

        // Check if file is empty
        config_file.seekg(0, std::ios::end);
        if (config_file.tellg() == 0) {
            config = nlohmann::json::object();
            std::cout << "Config file is empty, initialized with empty JSON object." << std::endl;
            return true;
        }
        config_file.seekg(0, std::ios::beg);

        try {
            config_file >> config;
            std::cout<<"Successfully loaded camera config from:  "<<config_abs_path.c_str()<<std::endl;
            return true;
        } catch (const nlohmann::json::exception& e) {
            std::cout<<"Failed to parse config file! Path:   "<<config_abs_path.c_str()<<"  "<<e.what()<<std::endl;
            return false;
        }
    }
}

void cameraCalibration::setMarkType(const std::string& mark_type) {
    if (mark_type == "TAG16h5") {
        detector_mark_.setMarkType(TAG16h5);
    } else if (mark_type == "TAG25h7") {
        detector_mark_.setMarkType(TAG25h7);
    } else if (mark_type == "TAG25h9") {
        detector_mark_.setMarkType(TAG25h9);
    } else if (mark_type == "TAG36h11") {
        detector_mark_.setMarkType(TAG36h11);
    } else if (mark_type == "TAG36h10") {
        detector_mark_.setMarkType(TAG36h10);
    } else if (mark_type == "ARUCO_MIP_36h12") {
        detector_mark_.setMarkType(ARUCO_MIP_36h12);
    } else if (mark_type == "ARUCO") {
        detector_mark_.setMarkType(ARUCO);
    } else if (mark_type == "ARUCO_MIP_25h7") {
        detector_mark_.setMarkType(ARUCO_MIP_25h7);
    } else if (mark_type == "ARUCO_MIP_16h3") {
        detector_mark_.setMarkType(ARUCO_MIP_16h3);
    } else if (mark_type == "ARTAG") {
        detector_mark_.setMarkType(ARTAG);
    } else if (mark_type == "ARTOOLKITPLUS") {
        detector_mark_.setMarkType(ARTOOLKITPLUS);
    } else if (mark_type == "ARTOOLKITPLUSBCH") {
        detector_mark_.setMarkType(ARTOOLKITPLUSBCH);
    } else if (mark_type == "CHILITAGS") {
        detector_mark_.setMarkType(CHILITAGS);
    } else if (mark_type == "CUSTOM") {
        detector_mark_.setMarkType(CUSTOM);
    } else if (mark_type == "MY_TEST") {
        detector_mark_.setMarkType(MY_TEST);
    } else {
        detector_mark_.setMarkType(MY_TEST);
    }
}

void cameraCalibration::set_workspace_id(const std::string& workspace_id) {
    workspace_id_ = workspace_id;
}

void cameraCalibration::set_save_path(const std::string& save_path) {
    save_path_ = save_path;
}



void cameraCalibration::set_hand_eye_type(int hand_eye_type) {
    hand_eye_type_ = hand_eye_type;
    LOG_INFO("Set hand eye type: {}", hand_eye_type_);
}

void cameraCalibration::set_config_dir(std::string config_dir) {
    mark3d_.init(config_dir);
    mark3d_.read_param_json();
}
