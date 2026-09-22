#include <rclcpp/rclcpp.hpp>
#include "cameraClient.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <thread>

std::string get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time_t), "%Y_%m_%d_%H_%M_%S");
    ss << "_" << std::setfill('0') << std::setw(3) << now_ms.count();
    return ss.str();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("camera_test_node");
    
    cameraClient cam_client;
    cam_client.imageClient(node, "color_dep_intrinsic");
    
    RCLCPP_INFO(node->get_logger(), "Camera test client started");
    
    // 发送请求
    std::string camera_id = "camera1";
    if (argc > 1) {
        camera_id = argv[1];
    }
    
    RCLCPP_INFO(node->get_logger(), "Sending request for camera: %s", camera_id.c_str());
    cam_client.send_request(camera_id);
    
    // 在单独线程中spin
    std::atomic<bool> running(true);
    std::thread spin_thread([&node, &running]() {
        while (running && rclcpp::ok()) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    
    // 获取图像
    cv::Mat color, dep;
    std::vector<float> intrinsic;
    float scale;
    int error_data;
    
    bool success = cam_client.get_camera_img(color, dep, intrinsic, scale, error_data);
    
    running = false;
    if (spin_thread.joinable()) {
        spin_thread.join();
    }
    
    if (success) {
        RCLCPP_INFO(node->get_logger(), "Got image successfully!");
        RCLCPP_INFO(node->get_logger(), "Color: %d x %d, Depth: %d x %d", 
                    color.cols, color.rows, dep.cols, dep.rows);
        RCLCPP_INFO(node->get_logger(), "Intrinsic: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                    intrinsic[0], intrinsic[4], intrinsic[2], intrinsic[5]);
        RCLCPP_INFO(node->get_logger(), "Scale: %.4f", scale);
        
        // 保存图像
        std::string timestamp = get_timestamp();
        std::string color_path = "/tmp/color_" + timestamp + ".png";
        std::string depth_path = "/tmp/depth_" + timestamp + ".png";
        
        cv::imwrite(color_path, color);
        cv::imwrite(depth_path, dep);
        
        RCLCPP_INFO(node->get_logger(), "Images saved to:");
        RCLCPP_INFO(node->get_logger(), "  Color: %s", color_path.c_str());
        RCLCPP_INFO(node->get_logger(), "  Depth: %s", depth_path.c_str());
        
        // 保存内参到文件
        std::string intrinsic_path = "/tmp/intrinsic_" + timestamp + ".txt";
        std::ofstream ofs(intrinsic_path);
        ofs << "fx: " << intrinsic[0] << "\n";
        ofs << "fy: " << intrinsic[4] << "\n";
        ofs << "cx: " << intrinsic[2] << "\n";
        ofs << "cy: " << intrinsic[5] << "\n";
        ofs << "scale: " << scale << "\n";
        ofs.close();
        RCLCPP_INFO(node->get_logger(), "  Intrinsic: %s", intrinsic_path.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to get image, error code: %d", error_data);
    }
    
    rclcpp::shutdown();
    return success ? 0 : -1;
}
