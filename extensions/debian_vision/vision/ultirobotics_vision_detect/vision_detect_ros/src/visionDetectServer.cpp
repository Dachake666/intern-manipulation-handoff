#include "visionDetectServer.h"
#include <functional>
#include <filesystem>
#include <fstream>
#include "geometry_utils.h"

visionDetectServer::visionDetectServer() {
    camera_manager_ = std::make_shared<cameraClientManager>();
}

visionDetectServer::~visionDetectServer() {
    stop();
}

void visionDetectServer::get_rt_inv(std::vector<std::vector<double>>& rt, std::vector<double>& rt_inv) {
    cv::Mat rt_M = cv::Mat(4, 4, CV_64FC1);
    double* rt_data = (double*)rt_M.data;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            rt_data[i * 4 + j] = rt[i][j];
        }
    cv::Mat rt_inv_M = rt_M.inv();
    double* inv_data = (double*)rt_inv_M.data;
    rt_inv.resize(12);
    for (int i = 0; i < 12; ++i) {
        rt_inv[i] = inv_data[i];
    }
}

std::string visionDetectServer::get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time_t), "%Y_%m_%d_%H_%M_%S");
    ss << "_" << std::setfill('0') << std::setw(3) << now_ms.count();
    return ss.str();
}

std::string visionDetectServer::get_date() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time_t), "%Y_%m_%d");
    return ss.str();
}
void visionDetectServer::save_request_json(std::string response_file, request_data& data){
    try{
        nlohmann::json json;
        json["workspace_id"] = data.workspace_id;
        json["request_detect"] = data.request_detect;
        json["text_prompt"] = data.text_prompt;
        json["object_size"] = data.object_size;
        json["hand_poses"] = data.hand_poses;
        
        std::ofstream file(response_file);
         if (file.is_open()) {
            file << json.dump(2);
            file.close();
            RCLCPP_INFO(node_->get_logger(), "Saved request json: %s", response_file.c_str());
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to save request json: %s", response_file.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception in save_request_json: %s", e.what());
    }
}
void visionDetectServer::save_request_data(std::string path, std::string time_date, std::string time_ms, request_data data) {
    try {
        std::string save_path_success = path + "/" + time_date;
        std::string save_path_fail = path + "/failed";
        
        if (!std::filesystem::exists(save_path_success)) {
            std::filesystem::create_directories(save_path_success);
        }        
        std::string response_file = save_path_success + "/" + time_ms + "_request.json";
        save_request_json(response_file, data);
        //if(!data.success){
        //    if (!std::filesystem::exists(save_path_fail)) {
        //        std::filesystem::create_directories(save_path_fail);
        //    }
        //    std::string response_file_fail = save_path_fail + "/" + time_ms + "_request.json";
        //    save_request_json(response_file_fail, data);
        //}
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception in save_request_json: %s", e.what());
    }
}

void visionDetectServer::save_json(std::string response_file, response_data& data) {
    try {              
        nlohmann::ordered_json json;
        json["success"] = data.success;
        json["count"] = data.count;
        json["error_data"] = data.error_data;
        json["material_pt_size"] = data.material_pt_size;
        
        nlohmann::ordered_json objects_json = nlohmann::ordered_json::array();
        for (const auto& obj : data.object) {
            nlohmann::ordered_json obj_json;
            obj_json["index"] = obj.index;
            obj_json["position_x"] = obj.position_x;
            obj_json["position_y"] = obj.position_y;
            obj_json["position_z"] = obj.position_z;
            obj_json["orientation_x"] = obj.orientation_x;
            obj_json["orientation_y"] = obj.orientation_y;
            obj_json["orientation_z"] = obj.orientation_z;
            obj_json["orientation_w"] = obj.orientation_w;
            obj_json["area"] = obj.area;
            obj_json["length"] = obj.length;
            obj_json["width"] = obj.width;
            obj_json["height"] = obj.height;
            obj_json["overlap_count"] = obj.overlap_count;
            obj_json["overlap_ratio"] = obj.overlap_ratio;
            obj_json["overlap_index"] = obj.overlap_index;
            
            std::vector<float> outside_polygon_vertex(3*obj.outside_polygon_vertex.size());
            for(int i = 0; i < obj.outside_polygon_vertex.size(); i++){
                outside_polygon_vertex[i*3] = obj.outside_polygon_vertex[i].x;
                outside_polygon_vertex[i*3+1] = obj.outside_polygon_vertex[i].y;
                outside_polygon_vertex[i*3+2] = obj.outside_polygon_vertex[i].z;
            }
            obj_json["outside_polygon_vertex"] = outside_polygon_vertex;
            objects_json.push_back(obj_json);
        }
        json["object"] = objects_json;
        
        nlohmann::ordered_json material_points_json = nlohmann::ordered_json::array();
        for (const auto& pt : data.material_points) {
            nlohmann::ordered_json pt_json;
            pt_json["x"] = pt.x;
            pt_json["y"] = pt.y;
            pt_json["z"] = pt.z;
            material_points_json.push_back(pt_json);
        }
        json["material_points"] = material_points_json;
        json["hand_and_eye"] = data.hand_and_eye;
        json["hand_pose"] = data.hand_pose;
        json["box_crop_pose"] = data.box_crop_pose;
        json["box_crop_size"] = data.box_crop_size;
        
        std::ofstream file(response_file);
        if (file.is_open()) {
            file << json.dump(2);
            file.close();
            RCLCPP_INFO(node_->get_logger(), "Saved response json: %s", response_file.c_str());
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to save response json: %s", response_file.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception in save_response_json: %s", e.what());
    }
}


void visionDetectServer::save_response_json(std::string path, std::string time_date, std::string time_ms, response_data data) {
    
    try {
        std::string save_path_success = path + "/" + time_date;
        std::string save_path_fail = path + "/failed";
        
        if (!std::filesystem::exists(save_path_success)) {
            std::filesystem::create_directories(save_path_success);
        }        
        std::string response_file = save_path_success + "/" + time_ms + "_response.json";
        save_json(response_file, data);
        if(!data.success){
            if (!std::filesystem::exists(save_path_fail)) {
                std::filesystem::create_directories(save_path_fail);
            }
            std::string response_file_fail = save_path_fail + "/" + time_ms + "_response.json";
            save_json(response_file_fail, data);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception in save_response_json: %s", e.what());
    }

}

void read_algorithm_param(std::string config_file, algorithmParam& param) {
    nlohmann::json json;
    if (!vision::utils::read_json(config_file, json)) {
        std::cout<<"Exception in read_algorithm_param: "<<config_file<<std::endl;
        return;
    }
    auto config_dir = std::filesystem::path(config_file).parent_path().string();
    if (json.contains("material_box_param") && json["material_box_param"].is_object()) {
        const auto& box = json["material_box_param"];
        if (box.contains("length")) {
            param.material_box.box_length = box["length"].get<int>();
        }
        if (box.contains("width")) {
            param.material_box.box_width = box["width"].get<int>();
        }
        if (box.contains("is_using_size")) {
            param.material_box.is_using_box_size = box["is_using_size"].get<bool>();
        }
        if (box.contains("is_detect")) {
            param.material_box.is_detect = box["is_detect"].get<bool>();
        }
        if (box.contains("img_position_x")) {
            param.material_box.img_position.x = box["img_position_x"].get<float>();
        }
        if (box.contains("img_position_y")) {
            param.material_box.img_position.y = box["img_position_y"].get<float>();
        }
    }
    if (json.contains("above_point_limit") && json["above_point_limit"].is_object()) {
        const auto& point_limit = json["above_point_limit"];
        if (point_limit.contains("Z_Range")) {
            param.box_Z_Range = point_limit["Z_Range"].get<int>();
        }
        if (point_limit.contains("PointLimit")) {
            param.box_PointLimit = point_limit["PointLimit"].get<int>();
        }
    }
    if (json.contains("background_image") && json["background_image"].is_object()) {
        const auto& back_ground = json["background_image"];
        if (back_ground.contains("is_using")) {
            param.back_ground_img.is_using = back_ground["is_using"].get<bool>();
        }
        if (back_ground.contains("path")) {
            std::string tmp_path = back_ground["path"].get<std::string>();
            param.back_ground_img.path = config_dir + "/" + tmp_path;
        }
        if (json.contains("result_candidate_number")) {
            param.result_candidate_number = json["result_candidate_number"].get<int>();
        }
    }
    if (json.contains("detect_rio_2d") && json["detect_rio_2d"].is_object()) {
        const auto& rio = json["detect_rio_2d"];
        if (rio.contains("is_using")) {
            param.seg_rio.is_using = rio["is_using"].get<bool>();
        }
        if (rio.contains("rio")) {
            std::vector<std::vector<float>> rio_points;
            rio_points = rio["rio"].get<std::vector<std::vector<float>>>();
            param.seg_rio.pts.clear();
            for (int i = 0; i < rio_points.size(); ++i) {
                if (rio_points[i].size() != 2) {
                    std::cout << "detect_rio_2d_points_error!!" << std::endl;
                    continue;
                }
                param.seg_rio.pts.emplace_back(rio_points[i][0], rio_points[i][1]);
            }
        }
        if (rio.contains("inside_ratio")) {
            param.seg_rio.inside_ratio = rio["inside_ratio"].get<float>();
        }
    }
}
bool visionDetectServer::init(rclcpp::Node::SharedPtr node, const std::string& config_path) {
    node_ = node;
    
    RCLCPP_INFO(node->get_logger(), "read config file: %s", config_path.c_str());
    nlohmann::json json;
    if (!vision::utils::read_json(config_path, json)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to read config file: %s", config_path.c_str());
        return false;
    }
    
    auto config_dir = std::filesystem::path(config_path).parent_path().string();
    
    try {
        if (!json.contains("workspaces")) {
            RCLCPP_ERROR(node->get_logger(), "Config file missing 'workspaces' field");
            return false;
        }
        std::string save_path;
        std::string save_image_type = "png";
        bool save_color = true;
        bool save_depth = true;
        bool save_pred = true;

        if(json.contains("archive")){
            if(json["archive"].contains("save_path")){
                save_path = json["archive"]["save_path"];
            } else {
                RCLCPP_WARN(node->get_logger(), "json archive missing 'save_path'");
            }
            
            if (json["archive"].contains("save_image_type")) {
                save_image_type = json["archive"]["save_image_type"];
            } else {
                RCLCPP_WARN(node->get_logger(), "json archive missing 'save_image_type'");
            }
            
            if (json["archive"].contains("save_color")) {
                save_color = json["archive"]["save_color"].get<bool>();
            } else {
                RCLCPP_WARN(node->get_logger(), "json archive missing 'save_color'");
            }
                
            if (json["archive"].contains("save_depth")) {
                save_depth = json["archive"]["save_depth"].get<bool>();
            } else {
                RCLCPP_WARN(node->get_logger(), "json archive missing 'save_depth'");
            }
            
            if (json["archive"].contains("save_pred")) {
                save_pred = json["archive"]["save_pred"].get<bool>();
            } else {
                RCLCPP_WARN(node->get_logger(), "json archive missing 'save_pred'");
            }
            
        } else {
            RCLCPP_WARN(node->get_logger(), "json archive missing");
        }
        RCLCPP_INFO(node->get_logger(), "save_path: %s, save_image_type: %s, save_color: %d, save_depth: %d, save_pred: %d", save_path.c_str(), save_image_type.c_str(), save_color, save_depth, save_pred);
        save_path_ = save_path;
        
        for (const auto& workspace : json["workspaces"]) {
            std::string workspace_id;
            if (workspace.contains("workspace_id")) {
                workspace_id = workspace["workspace_id"];
            } else {
                RCLCPP_ERROR(node->get_logger(), "Workspace missing 'workspace_id' field, skipping");
                continue;
            }
            
            std::string camera_id;
            if (workspace.contains("camera_id")) {
                camera_id = workspace["camera_id"];
            } else {
                RCLCPP_ERROR(node->get_logger(), "Workspace [%s] missing 'camera_id' field", workspace_id.c_str());
            }
            
            workspace_camera_map_[workspace_id] = camera_id;
            
            algorithmParam param;
            param.workstation = workspace_id;
            
            if (workspace.contains("model_path")) {
                std::string model_name = workspace["model_path"];
                std::string model_path = config_dir +"/models/" + model_name;
                RCLCPP_INFO(node->get_logger(), "read model_path : %s", model_path.c_str());
                param.model_path = model_path;
            } else {
                RCLCPP_ERROR(node->get_logger(), "Workspace [%s] missing 'model_path', using default: %s", 
                            workspace_id.c_str(), param.model_path.c_str());
            }            
            param.result_data_path = save_path;
            param.save_image_type = save_image_type;
            param.save_color = save_color;
            param.save_depth = save_depth;
            param.save_pred = save_pred;
           
            if (workspace.contains("hand_and_eye")) {
                param.hand_and_eye = workspace["hand_and_eye"];
            } else {
                RCLCPP_ERROR(node->get_logger(), "Workspace [%s] missing 'hand_and_eye', using default: %d", 
                            workspace_id.c_str(), param.hand_and_eye);
            }
            if(workspace.contains("sam3_server_ip")){
                param.sam3_server_ip = workspace["sam3_server_ip"];
            } else {
                RCLCPP_ERROR(node->get_logger(), "Workspace [%s] missing 'sam3_server_ip', using default: %s", 
                            workspace_id.c_str(), param.sam3_server_ip.c_str());
            }
            
            if (workspace.contains("hand_eye_matrix")) {
                param.camera_rt = workspace["hand_eye_matrix"];
                param.camera_rt_eye = param.camera_rt;
                get_rt_inv(param.camera_rt, param.rt_inv);
            } else {
                RCLCPP_ERROR(node->get_logger(), "Workspace [%s] missing 'hand_eye_matrix', using default identity matrix", 
                            workspace_id.c_str());
            }
            if (workspace.contains("crop_region") && workspace["crop_region"].is_object()) {
                const auto& crop_region = workspace["crop_region"];
                std::vector<float> size;
                std::vector<float> position;
                if (crop_region.contains("is_using") && crop_region["is_using"].is_number_integer()) {
                    param.is_using_box_crop = crop_region["is_using"].get<int>();
                }
                if (crop_region.contains("pose") && crop_region["pose"].is_array()) {
                    position = crop_region["pose"].get<std::vector<float>>();
                }
                if (crop_region.contains("size") && crop_region["size"].is_array()) {
                    size = crop_region["size"].get<std::vector<float>>();
                }
                if (size.size() > 2&& position.size()>5) {
                    param.box_crop_pose = position;
                    param.box_crop_size.x = size[0];
                    param.box_crop_size.y = size[1];
                    param.box_crop_size.z = size[2];
                }
            }
            if (workspace.contains("algorithm_parameter")) {
                std::string param_name = workspace["algorithm_parameter"];
                std::string algorithm_file_name = config_dir +"/algorithm/" + param_name+".json";
                RCLCPP_INFO(node->get_logger(), "read algorithm_file_name : %s", algorithm_file_name.c_str());
                read_algorithm_param(algorithm_file_name, param);
            }
            workspace_param_map_[workspace_id] = param;
            
            auto detector = std::make_shared<visionObjectDetect>(workspace_param_map_[workspace_id]);
            workspace_detector_map_[workspace_id] = detector;
            
            RCLCPP_INFO(node->get_logger(), "Initialized workspace: %s with camera: %s", 
                        workspace_id.c_str(), camera_id.c_str());
        }
        
    } catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "JSON parse error: %s", e.what());
        return false;
    }
    
    camera_manager_->init(node, "color_dep_intrinsic");
    
    startServer(node, "vision_detect");
    RCLCPP_INFO(node->get_logger(), "Vision detect server initialized successfully!");
    return true;
}

void visionDetectServer::startServer(rclcpp::Node::SharedPtr node, const std::string& action_name) {
    detect_server_ = rclcpp_action::create_server<VisionDetectResult>(
        node,
        action_name,
        std::bind(&visionDetectServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&visionDetectServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&visionDetectServer::executeCallback, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse visionDetectServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const VisionDetectResult::Goal> goal) {
    RCLCPP_INFO(node_->get_logger(), "Received goal request for workspace: %s", goal->workspace_id.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse visionDetectServer::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<VisionDetectResult>> goal_handle) {
    RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void visionDetectServer::stop() {
    detect_server_.reset();
    workspace_detector_map_.clear();
    workspace_param_map_.clear();
    workspace_camera_map_.clear();
    camera_manager_.reset();
}

void visionDetectServer::set_response_data(
    std::shared_ptr<Result> result,
    std::vector<object_pose>& poses,
    int error_data,
    algorithmParam& param) {
    
    result->error_data = error_data;
    save_response_data_.error_data = error_data;
    save_response_data_.hand_and_eye = param.hand_and_eye;
    save_response_data_.hand_pose = param.hand_pose;
    save_response_data_.box_crop_pose = param.box_crop_pose;
    std::vector<float> box_crop_size = {param.box_crop_size.x, param.box_crop_size.y, param.box_crop_size.z};
    save_response_data_.box_crop_size = box_crop_size;
    if (error_data == 0 && poses.size() > 0) {
        int count = poses.size();
        std::vector<vision_interface::msg::VisionObject> object_pose_msgs(count);
        save_response_data_.object.resize(count);

        for (size_t i = 0; i < poses.size(); ++i) {
            Eigen::Quaterniond tem_qua = xyzEulerDegToQuaternion(poses[i].angle_x, poses[i].angle_y, poses[i].angle_z);
            
            object_pose_msgs[i].center_pose.position.x = poses[i].x;
            object_pose_msgs[i].center_pose.position.y = poses[i].y;
            object_pose_msgs[i].center_pose.position.z = poses[i].z;
            object_pose_msgs[i].center_pose.orientation.x = tem_qua.x();
            object_pose_msgs[i].center_pose.orientation.y = tem_qua.y();
            object_pose_msgs[i].center_pose.orientation.z = tem_qua.z();
            object_pose_msgs[i].center_pose.orientation.w = tem_qua.w();
            object_pose_msgs[i].angle.x = poses[i].angle_x;
            object_pose_msgs[i].angle.y = poses[i].angle_y;
            object_pose_msgs[i].angle.z = poses[i].angle_z;
            
            object_pose_msgs[i].area = poses[i].area;
            
            object_pose_msgs[i].overlap.overlap_count = poses[i].overlap_index.size();
            object_pose_msgs[i].overlap.overlap_ratio = poses[i].overlap_area_ratio;
            object_pose_msgs[i].overlap.overlap_index = poses[i].overlap_index;
            
            object_pose_msgs[i].size.x = poses[i].length;
            object_pose_msgs[i].size.y = poses[i].width;
            object_pose_msgs[i].size.z = poses[i].height;
            object_pose_msgs[i].outside.point_number = poses[i].polygon_vertex.size();
            object_pose_msgs[i].outside.points.resize(poses[i].polygon_vertex.size());
            for (size_t j = 0; j < poses[i].polygon_vertex.size(); ++j) {
                object_pose_msgs[i].outside.points[j].x = poses[i].polygon_vertex[j].x;
                object_pose_msgs[i].outside.points[j].y = poses[i].polygon_vertex[j].y;
                object_pose_msgs[i].outside.points[j].z = poses[i].polygon_vertex[j].z;
            }
            object_pose_msgs[i].inside.point_number = 0;

            save_response_data_.object[i].position_x = poses[i].x;
            save_response_data_.object[i].position_y = poses[i].y;
            save_response_data_.object[i].position_z = poses[i].z;
            save_response_data_.object[i].orientation_x = tem_qua.x();
            save_response_data_.object[i].orientation_y = tem_qua.y();
            save_response_data_.object[i].orientation_z = tem_qua.z();
            save_response_data_.object[i].orientation_w = tem_qua.w();
            save_response_data_.object[i].area = poses[i].area;
            save_response_data_.object[i].length = poses[i].length;
            save_response_data_.object[i].width = poses[i].width;
            save_response_data_.object[i].height = poses[i].height;
            save_response_data_.object[i].overlap_count = poses[i].overlap_index.size();
            save_response_data_.object[i].overlap_ratio = poses[i].overlap_area_ratio;
            save_response_data_.object[i].overlap_index = poses[i].overlap_index;
            save_response_data_.object[i].outside_polygon_vertex = poses[i].polygon_vertex;
            save_response_data_.object[i].index = i;
        }
        
        result->success = true;
        result->count = count;
        result->object_pose = object_pose_msgs;

        save_response_data_.success = true;
        save_response_data_.count = count;
        
        int material_pt_size = param.material_box.material_box_points.size();
        result->material_pt_size = material_pt_size;
        save_response_data_.material_pt_size = material_pt_size;
        
        if (material_pt_size > 0) {
            std::vector<geometry_msgs::msg::Point> material_points(material_pt_size);
            save_response_data_.material_points.resize(material_pt_size);
            
            for (size_t i = 0; i < param.material_box.material_box_points.size(); ++i) {
                material_points[i].x = param.material_box.material_box_points[i].x;
                material_points[i].y = param.material_box.material_box_points[i].y;
                material_points[i].z = param.material_box.material_box_points[i].z;
                
                save_response_data_.material_points[i].x = param.material_box.material_box_points[i].x;
                save_response_data_.material_points[i].y = param.material_box.material_box_points[i].y;
                save_response_data_.material_points[i].z = param.material_box.material_box_points[i].z;
            }
            result->material_points = material_points;
        }
    } else {
        result->success = false;
        result->count = 0;

        save_response_data_.success = false;
        save_response_data_.count = 0;
    }
}

void visionDetectServer::executeCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<VisionDetectResult>> goal_handle) {
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<VisionDetectResult::Result>();
    
    std::string time_date = get_date();
    std::string time_ms = get_timestamp();
    save_response_data_.clear();
    save_request_data_.request_detect = goal->request_detect;
    save_request_data_.text_prompt = goal->text_prompt;
    save_request_data_.workspace_id = goal->workspace_id;
    save_request_data_.object_size.assign(goal->object_size.begin(), goal->object_size.end());
    save_request_data_.hand_poses.resize(7);
    save_request_data_.hand_poses[0] = goal->hand_pose.position.x;    
    save_request_data_.hand_poses[1] = goal->hand_pose.position.y;    
    save_request_data_.hand_poses[2] = goal->hand_pose.position.z;    
    save_request_data_.hand_poses[3] = goal->hand_pose.orientation.x;    
    save_request_data_.hand_poses[4] = goal->hand_pose.orientation.y;       
    save_request_data_.hand_poses[5] = goal->hand_pose.orientation.z;    
    save_request_data_.hand_poses[6] = goal->hand_pose.orientation.w;    
    
    std::string workspace_id = goal->workspace_id;
    std::string write_dir_response = save_path_ + "/" + workspace_id;
    std::thread request_thread(&visionDetectServer::save_request_data, this, write_dir_response, time_date, time_ms, save_request_data_);
    request_thread.detach();
    
    RCLCPP_INFO(node_->get_logger(), "Executing goal for workspace: %s", workspace_id.c_str());
    
    if (workspace_detector_map_.find(workspace_id) == workspace_detector_map_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Unknown workspace: %s", workspace_id.c_str());
        result->success = false;
        result->error_data = -51;
        result->count = 0;
        goal_handle->succeed(result);
        save_response_data_.success = result->success;
        save_response_data_.error_data = result->error_data;
        save_response_data_.count = result->count;
        std::thread response_thread(&visionDetectServer::save_response_json, this, write_dir_response, time_date, time_ms, save_response_data_);
        response_thread.detach();
        return;
    }
    
    std::string camera_id = workspace_camera_map_[workspace_id];
    algorithmParam& param = workspace_param_map_[workspace_id];
    auto detector = workspace_detector_map_[workspace_id];

    if(1 == param.hand_and_eye){
        param.hand_pose[0] = goal->hand_pose.position.x;
        param.hand_pose[1] = goal->hand_pose.position.y;
        param.hand_pose[2] = goal->hand_pose.position.z;
        param.hand_pose[3] = goal->hand_pose.orientation.x;
        param.hand_pose[4] = goal->hand_pose.orientation.y;
        param.hand_pose[5] = goal->hand_pose.orientation.z;
        param.hand_pose[6] = goal->hand_pose.orientation.w;
    }
    
    camera_callback_data cam_data;

    int detect_type = goal->request_detect;
    switch (detect_type) {
    case -100:
        break;
    case -99:
        break;
    case -1: {
        RCLCPP_INFO(node_->get_logger(), "detect_type=-1,local_img_detect");
        detector->read_local_image_data(cam_data);
        if(cam_data.error_data != 0 || cam_data.color.empty()||cam_data.dep.empty()){
            RCLCPP_ERROR(node_->get_logger(), "Failed to load local image data: %d", cam_data.error_data);
            result->success = false;
            result->error_data = cam_data.error_data;
            result->count = 0;
            goal_handle->succeed(result);
            save_response_data_.success = result->success;
            save_response_data_.error_data = result->error_data;
            save_response_data_.count = result->count;
            std::thread response_thread(&visionDetectServer::save_response_json, this, write_dir_response, time_date,time_ms, save_response_data_);
            response_thread.detach();
            return;
        }
        auto feedback_local = std::make_shared<VisionDetectResult::Feedback>();
        feedback_local->status = "Local image data loaded";
        feedback_local->percent_complete = 50;
        goal_handle->publish_feedback(feedback_local);
        RCLCPP_INFO(node_->get_logger(), "Published feedback: 50%% - Local image data loaded");
        break;
    }
    case 1: {
        RCLCPP_INFO(node_->get_logger(), "detect_type=1,camera_data_detect");
        if (!camera_manager_->get_camera_image(camera_id, cam_data.color, cam_data.dep,cam_data.intrinsic, cam_data.scale, cam_data.error_data)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get camera image for %s", camera_id.c_str());
            //set_response_data(result, param.multiple_object_pose, cam_data.error_data, param);
            camera_manager_->release_camera(camera_id);
            result->success = false;
            result->error_data = cam_data.error_data;
            result->count = 0;
            goal_handle->succeed(result);
            save_response_data_.success = result->success;
            save_response_data_.error_data = result->error_data;
            save_response_data_.count = result->count;
            std::thread response_thread(&visionDetectServer::save_response_json, this, write_dir_response, time_date,time_ms, save_response_data_);
            response_thread.detach();
            return;
        }
        
        auto feedback_camera = std::make_shared<VisionDetectResult::Feedback>();
        feedback_camera->status = "Camera data captured";
        feedback_camera->percent_complete = 50;
        goal_handle->publish_feedback(feedback_camera);
        RCLCPP_INFO(node_->get_logger(), "Published feedback: 50%% - Camera data captured");
        break;
    }
    default:
        RCLCPP_ERROR(node_->get_logger(), "Unknown detect_type: %d", detect_type);
        break;
    }

    param.local_time_ms = time_ms;
    param.local_time_date = time_date;
    param.cam_data = cam_data;
    std::cout<<"local_time_ms: "<<param.local_time_ms<<std::endl;
    std::cout<<"local_time_date: "<<param.local_time_date<<std::endl;
    
    std::vector<std::string> text_prompts = goal->text_prompt;
    
    detector->objecgt_pose_detect(param.cam_data, text_prompts);
    detector->write_result_data();
    
    set_response_data(result, param.multiple_object_pose, param.result_error_data, param);
    
    std::thread response_thread(&visionDetectServer::save_response_json, this, write_dir_response, time_date,time_ms, save_response_data_);
    response_thread.detach();
    
    camera_manager_->release_camera(camera_id);
    
    goal_handle->succeed(result);
    
    if (param.result_error_data == 0 && param.multiple_object_pose.size() > 0) {
        RCLCPP_INFO(node_->get_logger(), "Goal succeeded, found %zu objects", param.multiple_object_pose.size());
    } else {
        RCLCPP_WARN(node_->get_logger(), "Goal succeeded but no objects found, error: %d", param.result_error_data);
    }
}
