#include"calibration_server.h"
#include <functional>
#include <opencv2/opencv.hpp>
#include <dirent.h>
using namespace std;
#include <nlohmann/json.hpp>
std::string log_name;
calibrationServer::calibrationServer(rclcpp::Node::SharedPtr node,std::string camera_server_name,std::string cal_server_name,std::string config_dir) {
    request_type_ = 0;
    
    // 创建独立的 callback_group 用于客户端，避免与服务端阻塞
    callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    hand_eye_type_ = 1;  
    loadConfig(config_dir);
    cam_calibration_.set_config_dir(config_dir);
    cam_calibration_.set_save_path(save_path_);
    cam_calibration_.setMarkType(mark_type_);
    cam_calibration_.set_hand_eye_type(hand_eye_type_);
    cam_client_.init(node, camera_server_name);
    createServer(node,cal_server_name);
}

calibrationServer::~calibrationServer() {
    stop();
}

void calibrationServer::stop() {
    vision_server_.reset();
    cam_client_.release();
}

void calibrationServer::get_camera_img(camera_callback_data& cam_data, const std::string& camera_id){
    cam_client_.get_camera_image(camera_id, cam_data.color,cam_data.dep,cam_data.intrinsic,cam_data.scale, cam_data.error_data);
    LOG_INFO("cam_data.scale {}",cam_data.scale );
}

void calibrationServer::createServer(rclcpp::Node::SharedPtr node,std::string cal_server_name) {
    // 服务端使用默认 callback_group（允许并发处理请求）
    vision_server_ = node->create_service<HandEyeCalibration>(
        cal_server_name,
        std::bind(&calibrationServer::handle_service, this, std::placeholders::_1, std::placeholders::_2));
}

void calibrationServer::handle_service(
    const std::shared_ptr<HandEyeCalibration::Request> request,
    std::shared_ptr<HandEyeCalibration::Response> response) {
    
    LOG_INFO("request_type_ !!!!  {}", request->request_type);
    LOG_INFO("camera_id !!!!  {}", request->camera_id);
    
    camera_callback_data cam_data;
    int error_data = 0;
    float total_rot_error = 0;
    float mean_trans_error = 0;
    std::vector<float> cam_rt;
    
    request_type_ = request->request_type;
    hand_pose_.x = request->hand_pose.position.x;
    hand_pose_.y = request->hand_pose.position.y;
    hand_pose_.z = request->hand_pose.position.z;
    hand_pose_.orientation_x = request->hand_pose.orientation.x;
    hand_pose_.orientation_y = request->hand_pose.orientation.y;
    hand_pose_.orientation_z = request->hand_pose.orientation.z;
    hand_pose_.orientation_w = request->hand_pose.orientation.w;
    std::string workspace_id = request->workspace_id;
    cam_calibration_.set_workspace_id(workspace_id);
    save_json_file_ = request->save_json_file;
    
    switch (request_type_) {
    case 1:
        get_camera_img(cam_data, request->camera_id);
        if(0!=cam_data.error_data || cam_data.color.empty() ||cam_data.dep.empty() ||cam_data.intrinsic.size()!=9){
            response->error_data = cam_data.error_data;
            response->success = false;
            break;
        }
        cam_calibration_.get_cam_img(cam_data,hand_pose_);
        error_data = cam_calibration_.get_error_data();
        response->error_data = error_data;
        response->success = false;
        if(0==error_data){
            response->success = true;
        }
        break;
    case 2:
        cam_calibration_.cam_calibration(save_json_file_,total_rot_error,mean_trans_error,cam_rt);
        error_data = cam_calibration_.get_error_data();
        response->error_data = error_data;
        response->mean_rot_error = total_rot_error;
        response->mean_trans_error = mean_trans_error;
        response->result_matrix = cam_rt;
        response->success = false;
        if(0==error_data){
            response->success = true;
        }
        break;
    case 3:
        cam_calibration_.cancel_last_detect();
        response->success = true;
        break;
    case -100:
    {
        std::vector<float> vec_hand_pose;
        pose_euler_angle local_hand_pose;
        local_data_read_.get_local_img(cam_data, vec_hand_pose);
        // 从vec_hand_pose直接读取位置和四元数 (共7个值: x, y, z, qx, qy, qz, qw)
        pose6d pose6d_hand;
        pose6d_hand.x = vec_hand_pose[0];
        pose6d_hand.y = vec_hand_pose[1];
        pose6d_hand.z = vec_hand_pose[2];
        pose6d_hand.orientation_x = vec_hand_pose[3];
        pose6d_hand.orientation_y = vec_hand_pose[4];
        pose6d_hand.orientation_z = vec_hand_pose[5];
        pose6d_hand.orientation_w = vec_hand_pose[6];
        cam_calibration_.get_cam_img(cam_data,pose6d_hand);
        response->error_data = error_data;
        response->success = false;
        if(0==error_data){
            response->success = true;
        }
        break;
    }
    default:
        response->error_data = -100;
        response->success = false;
        break;
    }
}
void calibrationServer::loadConfig(const std::string& config_dir) {
    mark_type_ = "MY_TEST";
    save_path_ = "save_cal_data";
	hand_eye_type_ = 1;
    if (config_dir.empty()) {
        LOG_WARN("Config directory is empty, using default values");
        mark_type_ = "MY_TEST";
        save_path_ = "save_cal_data";
        return;
    }

    std::string config_file = config_dir + "/config.json";
    if (access(config_file.c_str(), 0) != 0) {
        LOG_WARN("Config file not found: {}, using default values", config_file);
        mark_type_ = "MY_TEST";
        save_path_ = "save_cal_data";
        return;
    }

    try {
        std::ifstream ifs(config_file);
        if (ifs.is_open()) {
            nlohmann::json config_json;
            ifs >> config_json;

            if (config_json.contains("mark_type")) {
                mark_type_ = config_json["mark_type"].get<std::string>();
                LOG_INFO("Loaded mark_type: {}", mark_type_);
            }

            if (config_json.contains("save_path")) {
                save_path_ = config_json["save_path"].get<std::string>();
                LOG_INFO("Loaded save_path: {}", save_path_);
                log_name = save_path_ + "/calibration_server.log";
            }else{
                log_name = "save_data/calibration_server.log";
            }
			 Logger& logger = Logger::instance_logger();
			if (config_json.contains("hand_eye_type")) {
                hand_eye_type_ = config_json["hand_eye_type"].get<int>();
                LOG_INFO("Loaded hand_eye_type_: {}", hand_eye_type_);
            }

            ifs.close();
        }
    }
    catch (const std::exception& e) {
        LOG_ERROR("Failed to parse config file: {}", e.what());
        mark_type_ = "MY_TEST";
        save_path_ = "save_cal_data";
    }
}
