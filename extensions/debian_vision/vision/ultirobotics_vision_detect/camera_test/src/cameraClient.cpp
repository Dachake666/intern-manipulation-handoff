#include "cameraClient.h"

cameraClient::cameraClient() 
    : got_img_(false), is_getting_(false), is_get_img_success_(false), error_data_(-1), scale_(1.0f) {
}

cameraClient::~cameraClient() {
}

void cameraClient::imageClient(rclcpp::Node::SharedPtr node, std::string camera_server_name) {
    client_ptr_ = node->create_client<ColorDepIntrinsic>(camera_server_name);
}

void cameraClient::send_request(const std::string& camera_id) {
    if (is_getting_) {
        return;
    }
    
    is_getting_ = true;
    got_img_ = false;
    is_get_img_success_ = false;
    
    if (!client_ptr_->wait_for_service(std::chrono::seconds(2))) {
        error_data_ = -100;
        got_img_ = true;
        is_getting_ = false;
        m_cv_.notify_one();
        return;
    }
    
    auto request = std::make_shared<ColorDepIntrinsic::Request>();
    request->request_image = true;
    request->camera_id = camera_id;
    
    auto future_result = client_ptr_->async_send_request(
        request, std::bind(&cameraClient::response_callback, this, std::placeholders::_1));
}

void cameraClient::response_callback(rclcpp::Client<ColorDepIntrinsic>::SharedFuture future) {
    std::lock_guard<std::mutex> lock(m_mutex_);
    
    auto result = future.get();
    
    error_data_ = result->camera_connet_status;
    scale_ = result->scale;
    
    if (!result->success) {
        is_get_img_success_ = false;
        got_img_ = true;
        is_getting_ = false;
        m_cv_.notify_one();
        return;
    }
    
    int color_height = result->color_height;
    int color_width = result->color_width;
    int dep_height = result->dep_height;
    int dep_width = result->dep_width;
    int color_channels = result->color_channels;
    int color_pix_size = color_height * color_width;
    int dep_pix_size = dep_height * dep_width;
    
    if (result->color_data.size() != color_pix_size * color_channels ||
        result->dep_data.size() != dep_pix_size ||
        result->intrinsic.size() != 9) {
        error_data_ = -101;
        is_get_img_success_ = false;
        got_img_ = true;
        is_getting_ = false;
        m_cv_.notify_one();
        return;
    }
    
    if (color_channels == 1) {
        color_ = cv::Mat(color_height, color_width, CV_8UC1);
    } else if (color_channels == 3) {
        color_ = cv::Mat(color_height, color_width, CV_8UC3);
    }
    dep_ = cv::Mat(dep_height, dep_width, CV_16UC1);
    
    std::memcpy(color_.data, result->color_data.data(), color_pix_size * color_channels * sizeof(uint8_t));
    std::memcpy(dep_.data, result->dep_data.data(), dep_pix_size * sizeof(uint16_t));
    intrinsic_ = result->intrinsic;
    
    is_get_img_success_ = true;
    got_img_ = true;
    is_getting_ = false;
    m_cv_.notify_one();
}

bool cameraClient::get_camera_img(cv::Mat& color, cv::Mat& dep, std::vector<float>& intrinsic, float& scale, int& error_data) {
    std::unique_lock<std::mutex> lock(m_mutex_);
    m_cv_.wait_for(lock, std::chrono::seconds(10), [this] { return got_img_; });
    
    if (is_get_img_success_) {
        color = color_.clone();
        dep = dep_.clone();
        intrinsic = intrinsic_;
        scale = scale_;
    }
    
    error_data = error_data_;
    got_img_ = false;
    return is_get_img_success_;
}
