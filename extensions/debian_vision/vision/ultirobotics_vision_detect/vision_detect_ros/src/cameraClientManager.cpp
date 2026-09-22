#include "cameraClientManager.h"
#include <chrono>

cameraClientManager::cameraClientManager() {
}

cameraClientManager::~cameraClientManager() {
    camera_clients_.clear();
}

void cameraClientManager::init(rclcpp::Node::SharedPtr node, const std::string& service_name) {
    node_ = node;
    service_name_ = service_name;
    
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    RCLCPP_INFO(node_->get_logger(), "cameraClientManager initialized with service: %s", service_name.c_str());
}

bool cameraClientManager::get_camera_image(
    const std::string& camera_id,
    cv::Mat& color,
    cv::Mat& dep,
    std::vector<float>& intrinsic,
    float& scale,
    int& error_data,
    int timeout_ms) {
    
    std::shared_ptr<CameraClient> cam_client;
    
    {
        std::lock_guard<std::mutex> lock(manager_mutex_);
        if (camera_clients_.find(camera_id) == camera_clients_.end()) {
            cam_client = std::make_shared<CameraClient>();
            cam_client->client = node_->create_client<ColorDepIntrinsic>(
                service_name_,
                rmw_qos_profile_services_default,
                callback_group_);
            camera_clients_[camera_id] = cam_client;
            RCLCPP_INFO(node_->get_logger(), "Created client for camera: %s", camera_id.c_str());
        } else {
            cam_client = camera_clients_[camera_id];
        }
    }
    
    std::unique_lock<std::mutex> lock(cam_client->mutex);
    
    if (cam_client->in_use.load()) {
        RCLCPP_INFO(node_->get_logger(), "Camera %s is busy, waiting...", camera_id.c_str());
        if (!cam_client->cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
            [&cam_client] { return !cam_client->in_use.load(); })) {
            RCLCPP_WARN(node_->get_logger(), "Camera %s is busy, timeout waiting", camera_id.c_str());
            error_data = -44;
            return false;
        }
    }
    
    cam_client->in_use.store(true);
    cam_client->request_pending = true;
    cam_client->response_ready = false;
    
    RCLCPP_INFO(node_->get_logger(), "Waiting for camera service: %s", camera_id.c_str());
    
    if (!cam_client->client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "Camera service not available for %s", camera_id.c_str());
        cam_client->in_use.store(false);
        cam_client->cv.notify_all();
        error_data = -41;
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Camera service available, sending request for: %s", camera_id.c_str());
    
    auto request = std::make_shared<ColorDepIntrinsic::Request>();
    request->request_image = true;
    request->camera_id = camera_id;
    
    auto future_result = cam_client->client->async_send_request(
        request,
        [this, camera_id](rclcpp::Client<ColorDepIntrinsic>::SharedFuture future) {
            this->response_callback(camera_id, future);
        });
    
    RCLCPP_INFO(node_->get_logger(), "Request sent, waiting for response for: %s", camera_id.c_str());
    
    if (!cam_client->cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
        [&cam_client] { return cam_client->response_ready; })) {
        RCLCPP_ERROR(node_->get_logger(), "Camera %s request timeout", camera_id.c_str());
        cam_client->in_use.store(false);
        cam_client->cv.notify_all();
        error_data = -42;
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Got response for camera: %s", camera_id.c_str());
    
    if (cam_client->error_data != 0 || !cam_client->success) {
        error_data = cam_client->error_data;
        cam_client->in_use.store(false);
        cam_client->cv.notify_all();
        return false;
    }
    
    
    color = cam_client->color.clone();
    dep = cam_client->dep.clone();
    intrinsic = cam_client->intrinsic;
    scale = cam_client->scale;
    error_data = cam_client->error_data;
    
    return true;
}

void cameraClientManager::release_camera(const std::string& camera_id) {
    std::lock_guard<std::mutex> lock(manager_mutex_);
    
    if (camera_clients_.find(camera_id) != camera_clients_.end()) {
        auto& cam_client = camera_clients_[camera_id];
        cam_client->in_use.store(false);
        cam_client->cv.notify_all();
    }
}

void cameraClientManager::response_callback(
    const std::string& camera_id,
    rclcpp::Client<ColorDepIntrinsic>::SharedFuture future) {
    
    RCLCPP_INFO(node_->get_logger(), "Response callback for camera: %s", camera_id.c_str());
    
    std::shared_ptr<CameraClient> cam_client;
    
    {
        std::lock_guard<std::mutex> lock(manager_mutex_);
        if (camera_clients_.find(camera_id) == camera_clients_.end()) {
            return;
        }
        cam_client = camera_clients_[camera_id];
    }
    
    std::lock_guard<std::mutex> lock(cam_client->mutex);
    
    auto result = future.get();
    
    cam_client->error_data = result->camera_connet_status;
    cam_client->scale = result->scale;
    
    if (!result->success) {
        RCLCPP_ERROR(node_->get_logger(), "Camera %s returned failure", camera_id.c_str());
        cam_client->success = false;
        cam_client->response_ready = true;
        cam_client->cv.notify_all();
        return;
    }
    
    int color_height = result->color_height;
    int color_width = result->color_width;
    int dep_height = result->dep_height;
    int dep_width = result->dep_width;
    int color_channels = result->color_channels;
    
    if (result->color_data.empty() || result->dep_data.empty() || result->intrinsic.size() != 9) {
        RCLCPP_ERROR(node_->get_logger(), "Camera %s invalid data", camera_id.c_str());
        cam_client->error_data = -43;
        cam_client->success = false;
        cam_client->response_ready = true;
        cam_client->cv.notify_all();
        return;
    }
    
    if (color_channels == 1) {
        cam_client->color = cv::Mat(color_height, color_width, CV_8UC1);
    } else {
        cam_client->color = cv::Mat(color_height, color_width, CV_8UC3);
    }
    cam_client->dep = cv::Mat(dep_height, dep_width, CV_16UC1);
    
    std::memcpy(cam_client->color.data, result->color_data.data(), result->color_data.size());
    std::memcpy(cam_client->dep.data, result->dep_data.data(), result->dep_data.size() * sizeof(uint16_t));
    cam_client->intrinsic = result->intrinsic;
    
    RCLCPP_INFO(node_->get_logger(), "Camera %s image received: %dx%d", camera_id.c_str(), color_width, color_height);
    
    cam_client->success = true;
    cam_client->response_ready = true;
    cam_client->cv.notify_all();
}
