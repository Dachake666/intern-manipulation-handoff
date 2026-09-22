#include "cameraServer.h"
#include <functional>
#include <opencv2/opencv.hpp>
#include <vision_utils/io.h>

cameraServer::cameraServer(){
}

cameraServer::~cameraServer() {
    stop();
}

bool cameraServer::init(rclcpp::Node::SharedPtr node, const std::string& config_path) {
    nlohmann::json json;
    RCLCPP_INFO(node->get_logger(), "read camera_config file: %s", config_path.c_str());
    if (!vision::utils::read_json(config_path, json))
    {
      RCLCPP_ERROR(node->get_logger(), "read active camera config failed");
      return false;
    }

    std::set<std::string> camera_ids;
    try
    {
      for (const auto& cam : json["activate_cameras"])
      {
        camera_ids.emplace(cam);
      }
    }
    catch (const nlohmann::json::exception& e)
    {
      RCLCPP_ERROR(node->get_logger(), "parse active camera config error: [%s]", e.what());
      return false;
    }
    
    auto config_dir = std::filesystem::path(config_path).parent_path().string();
    
    for (const auto& id : camera_ids)
    {
      auto camera = std::make_shared<cameraRGBD>();      
      auto camera_config_path = config_dir + "/camera/" + id + "/config.json";
      RCLCPP_INFO(node->get_logger(), "read camera_config_path file: %s", camera_config_path.c_str());
      
      if (!camera->init(camera_config_path, id))
      {
        RCLCPP_ERROR(node->get_logger(), "init camera [%s] config failed", id.c_str());
        return false;
      }
      RCLCPP_INFO(node->get_logger(), "init camera [%s] success!", id.c_str());

      if (!camera->open())
      {
        RCLCPP_WARN(node->get_logger(), "open camera [%s] failed!", id.c_str());
        return false;
      }
      RCLCPP_INFO(node->get_logger(), "open camera [%s] success!", id.c_str());

      cameras_[id] = camera;
    }
    
    imageServer(node, "color_dep_intrinsic");
    RCLCPP_INFO(node->get_logger(), "camera server initialized successfully!");
    return true;
}

void cameraServer::stop() {
    for (auto& cam : cameras_){
        if (cam.second) {
            cam.second->close();
        }
    }
    cameras_.clear();
    vision_server_.reset();
}

void cameraServer::imageServer(rclcpp::Node::SharedPtr node, std::string camera_server_name) {
    vision_server_ = node->create_service<ColorDepIntrinsic>(
        camera_server_name,
        std::bind(&cameraServer::HandleService, this, std::placeholders::_1, std::placeholders::_2));
}

void cameraServer::HandleService(
    const std::shared_ptr<Request> request,
    std::shared_ptr<Response> response) {
    
    std::string camera_id = request->camera_id;
    cv::Mat color_img, dep_img;
    std::vector<float> intrinsic;
    
    if (cameras_.count(camera_id) && cameras_[camera_id]) {
        cameras_[camera_id]->get_cam_data(color_img, dep_img, intrinsic);
        response->camera_connet_status = cameras_[camera_id]->get_cam_state();
        response->scale = cameras_[camera_id]->get_cam_scale();
    } else {
        response->camera_connet_status = -30;
        response->scale = 0.25;
    }
    
    if(color_img.empty() || dep_img.empty() || intrinsic.size() != 9){
        response->success = false;
        response->color_height = 0;
        response->color_width = 0;
        response->dep_height = 0;
        response->dep_width = 0;
        if( 0 ==response->camera_connet_status)
            response->camera_connet_status = -31;
        return;
    }
    
    std::vector<uint8_t> color_data_vec;
    std::vector<uint16_t> dep_data_vec;
    color_data_vec.resize(color_img.cols * color_img.rows * color_img.channels());
    dep_data_vec.resize(dep_img.cols * dep_img.rows);

    memcpy(color_data_vec.data(), color_img.data, color_img.cols * color_img.rows * sizeof(uint8_t) * color_img.channels());
    memcpy(dep_data_vec.data(), dep_img.data, dep_img.cols * dep_img.rows * sizeof(uint16_t) * dep_img.channels());

    response->success = true;
    response->color_height = color_img.rows;
    response->color_width = color_img.cols;
    response->dep_height = dep_img.rows;
    response->dep_width = dep_img.cols;
    response->color_channels = color_img.channels();
    response->color_data = color_data_vec;
    response->dep_data = dep_data_vec;
    response->intrinsic = intrinsic;
}
