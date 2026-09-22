#include "geometry_utils.h"

float pointToLineDistance(const cv::Point2f& p0, const cv::Point2f& p1, const cv::Point2f& p2) {
    // 步骤1：计算直线一般式的系数A、B、C
    float A = p2.y - p1.y;
    float B = p1.x - p2.x;
    float C = p2.x * p1.y - p1.x * p2.y;

    // 步骤2：计算分母（A² + B²的平方根）
    float denominator = std::sqrt(A * A + B * B);

    // 异常处理：若p1和p2重合，分母为0，返回点到点的距离
    if (denominator < 1e-6) { // 浮点数精度误差，避免除以0
        float dx = p0.x - p1.x;
        float dy = p0.y - p1.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // 步骤3：计算分子的绝对值，再除以分母得到距离
    float numerator = A * p0.x + B * p0.y + C;
    return numerator / denominator;
}

Eigen::Quaterniond xyzEulerToQuaternion(double rx, double ry, double rz)
{
    // 1/2角度，用于四元数公式
    const double half_rx = rx * 0.5;
    const double half_ry = ry * 0.5;
    const double half_rz = rz * 0.5;

    // 三角函数计算
    const double crx = cos(half_rx);
    const double srx = sin(half_rx);
    const double cry = cos(half_ry);
    const double sry = sin(half_ry);
    const double crz = cos(half_rz);
    const double srz = sin(half_rz);

    // XYZ 内旋 四元数计算公式（与你的矩阵分解严格匹配）
    double w = crx * cry * crz + srx * sry * srz;
    double x = srx * cry * crz - crx * sry * srz;
    double y = crx * sry * crz + srx * cry * srz;
    double z = crx * cry * srz - srx * sry * crz;

    return Eigen::Quaterniond(w, x, y, z);
}

// 【便捷重载】直接输入角度值(度)，内部自动转弧度
Eigen::Quaterniond xyzEulerDegToQuaternion(double angleX_deg, double angleY_deg, double angleZ_deg)
{
    // 角度转弧度
    double rx = angleX_deg * M_PI / 180.0;
    double ry = angleY_deg * M_PI / 180.0;
    double rz = angleZ_deg * M_PI / 180.0;

    return xyzEulerToQuaternion(rx, ry, rz);
}

cv::Mat poseVectorToTransformMatrix_q(const std::vector<float>& pose)
{
    // 1. 检查输入长度是否正确
    if (pose.size() != 7)
    {
        CV_Error(cv::Error::StsBadArg, "pose vector must have 7 elements: x y z qx qy qz qw");
    }

    // 2. 提取平移和四元数
    float x = pose[0];
    float y = pose[1];
    float z = pose[2];
    float qx = pose[3];
    float qy = pose[4];
    float qz = pose[5];
    float qw = pose[6];

    // 3. 四元数转旋转矩阵
    cv::Mat rotMat(3, 3, CV_32F);

    // 四元数标准公式
    rotMat.at<float>(0, 0) = 1 - 2 * qy * qy - 2 * qz * qz;
    rotMat.at<float>(0, 1) = 2 * qx * qy - 2 * qw * qz;
    rotMat.at<float>(0, 2) = 2 * qx * qz + 2 * qw * qy;

    rotMat.at<float>(1, 0) = 2 * qx * qy + 2 * qw * qz;
    rotMat.at<float>(1, 1) = 1 - 2 * qx * qx - 2 * qz * qz;
    rotMat.at<float>(1, 2) = 2 * qy * qz - 2 * qw * qx;

    rotMat.at<float>(2, 0) = 2 * qx * qz - 2 * qw * qy;
    rotMat.at<float>(2, 1) = 2 * qy * qz + 2 * qw * qx;
    rotMat.at<float>(2, 2) = 1 - 2 * qx * qx - 2 * qy * qy;

    // 4. 构建 4x4 齐次变换矩阵
    cv::Mat transMat = cv::Mat::eye(4, 4, CV_32F);

    // 复制旋转矩阵到左上角 3x3
    rotMat.copyTo(transMat(cv::Rect(0, 0, 3, 3)));

    // 填入平移向量
    transMat.at<float>(0, 3) = x;
    transMat.at<float>(1, 3) = y;
    transMat.at<float>(2, 3) = z;

    return transMat;
}

bool is_point_in_polygon(const cv::Point2i& pt, const std::vector<cv::Point2f>& vertices) {
    if (vertices.size() < 3) {
        return false;
    }

    int count = 0;
    int n = vertices.size();

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