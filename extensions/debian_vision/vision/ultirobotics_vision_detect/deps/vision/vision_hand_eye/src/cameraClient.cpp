#include "cameraClient.h"
#include <chrono>
#include <functional>

cameraClient::cameraClient() {
    request_pending_ = false;
    response_ready_ = false;
    scale_ = 1;
    error_data_ = 0;
    success_ = false;
}

cameraClient::~cameraClient() {
}

void cameraClient::init(rclcpp::Node::SharedPtr node, const std::string& service_name) {
    node_ = node;
    service_name_ = service_name;
    
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    client_ = node_->create_client<ColorDepIntrinsic>(
        service_name_,
        rmw_qos_profile_services_default,
        callback_group_);
}

void cameraClient::response_callback(rclcpp::Client<ColorDepIntrinsic>::SharedFuture future) {
    auto result = future.get();
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    error_data_ = result->camera_connet_status;
    scale_ = result->scale;
    
    if (!result->success) {
        success_ = false;
        response_ready_ = true;
        cv_.notify_all();
        return;
    }
    
    int color_height = result->color_height;
    int color_width = result->color_width;
    int dep_height = result->dep_height;
    int dep_width = result->dep_width;
    int color_channels = result->color_channels;
    
    if (result->color_data.empty() || result->dep_data.empty() || result->intrinsic.size() != 9) {
        error_data_ = -102;
        success_ = false;
        response_ready_ = true;
        cv_.notify_all();
        return;
    }
    
    if (color_channels == 1) {
        color_ = cv::Mat(color_height, color_width, CV_8UC1);
    } else {
        color_ = cv::Mat(color_height, color_width, CV_8UC3);
    }
    dep_ = cv::Mat(dep_height, dep_width, CV_16UC1);
    
    std::memcpy(color_.data, result->color_data.data(), result->color_data.size());
    std::memcpy(dep_.data, result->dep_data.data(), result->dep_data.size() * sizeof(uint16_t));
    intrinsic_ = result->intrinsic;
    
    success_ = true;
    response_ready_ = true;
    cv_.notify_all();
}

bool cameraClient::get_camera_image(
    const std::string& camera_id,
    cv::Mat& color,
    cv::Mat& dep,
    std::vector<float>& intrinsic,
    float& scale,
    int& error_data,
    int timeout_ms) {
    
    std::unique_lock<std::mutex> lock(mutex_);
    
    request_pending_ = true;
    response_ready_ = false;
    
    lock.unlock();
    
    if (!client_->wait_for_service(std::chrono::seconds(2))) {
        lock.lock();
        error_data = -100;
        return false;
    }
    
    auto request = std::make_shared<ColorDepIntrinsic::Request>();
    request->request_image = true;
    request->camera_id = camera_id;  // 使用传入的 camera_id
    
    auto future_result = client_->async_send_request(
        request,
        [this](rclcpp::Client<ColorDepIntrinsic>::SharedFuture future) {
            this->response_callback(future);
        });
    
    lock.lock();
    
    if (!cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
        [this] { return response_ready_; })) {
        error_data = -101;
        return false;
    }
    
    if (error_data_ != 0 || !success_) {
        error_data = error_data_;
        return false;
    }
    
    color = color_.clone();
    dep = dep_.clone();
    intrinsic = intrinsic_;
    scale = scale_;
    error_data = error_data_;
    
    return true;
}

void cameraClient::release() {
    std::lock_guard<std::mutex> lock(mutex_);
    request_pending_ = false;
    response_ready_ = false;
    cv_.notify_all();
}