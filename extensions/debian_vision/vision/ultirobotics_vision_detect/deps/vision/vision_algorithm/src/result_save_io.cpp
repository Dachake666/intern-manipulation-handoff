#include"result_save_io.h"
#include"vision_utils/io.h"
//#include <windows.h>
//#include"io.h"

#ifdef _WIN32
#include <windows.h>
void find_color_png_files(const std::string& dirPath, std::vector<std::string>& file_names, std::string match_str1, std::string match_str2) {
    std::string searchPath = dirPath + "\\*";

    WIN32_FIND_DATAA findData;
    HANDLE hFind = FindFirstFileA(searchPath.c_str(), &findData);

    if (hFind == INVALID_HANDLE_VALUE) {
        std::cerr << "can_not_open_file:  " << dirPath << std::endl;
        return;
    }

    do {
        if (strcmp(findData.cFileName, ".") == 0 || strcmp(findData.cFileName, "..") == 0) {
            continue;
        }

        std::string tmp = findData.cFileName;
        if (tmp.find(match_str1) != -1 && tmp.find(match_str2) != -1)
            file_names.push_back(findData.cFileName);

    } while (FindNextFileA(hFind, &findData) != 0);

    if (GetLastError() != ERROR_NO_MORE_FILES) {
        std::cout << "search_file_error" << std::endl;
    }

    FindClose(hFind);
}
bool ensureDirectoryExists(const std::string& dirName) {
    // 1. 检查文件夹是否存在
    DWORD fileAttr = GetFileAttributesA(dirName.c_str());

    if (fileAttr == INVALID_FILE_ATTRIBUTES) {
        // 文件夹不存在，尝试创建
        BOOL createResult = CreateDirectoryA(dirName.c_str(), NULL);
        if (!createResult) {
            std::cerr << "创建文件夹 [" << dirName << "] 失败，错误码: " << GetLastError() << std::endl;
            return false;
        }
        std::cout << "文件夹 [" << dirName << "] 已创建" << std::endl;
    }
    else if (fileAttr & FILE_ATTRIBUTE_DIRECTORY) {
        // 文件夹已存在
        std::cout << "文件夹 [" << dirName << "] 已存在" << std::endl;
    }
    else {
        // 存在同名文件（非文件夹），无法创建
        //std::cerr << "错误：存在同名文件 [" << dirName << "]，无法创建文件夹" << std::endl;
        return false;
    }

    return true;
}

bool ensureDirectoryExistsRecursive(std::string fullDir) {

    std::string tp_fulldir = fullDir;
    std::string next_dir = "";
    while (tp_fulldir.length() > 0) {
        size_t sepPos = tp_fulldir.find_first_of("/\\");
        if (sepPos == std::string::npos) {
            return ensureDirectoryExists(tp_fulldir);
        }
        std::string parentDir = tp_fulldir.substr(0, sepPos);
        if (next_dir.length() < 1)
            next_dir = next_dir + parentDir;
        else
            next_dir = next_dir + "/" + parentDir;
        if (!ensureDirectoryExists(next_dir)) {
            return false;
        }

        std::string next_fulldir = tp_fulldir.substr(sepPos + 1, -1);
        tp_fulldir = next_fulldir;
    }
    return 1;
}

#else

bool is_target_file(const std::string& filename, std::string match1, std::string match2) {
    size_t color_pos = filename.find(match1);
    if (color_pos == std::string::npos) {
        return false;
    }
    size_t png_pos = filename.find(match2);
    if (png_pos == std::string::npos) {
        return false;
    }
    return true;
}


void find_color_png_files(const std::string& folder_path, std::vector<std::string>& result, std::string match1, std::string match2) {
    bool recursive = false;
    DIR* dir = opendir(folder_path.c_str());
    if (dir == NULL) {
        std::cout << "dir_NULL  " << folder_path << std::endl;
        return;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL) {
        std::string entry_name = entry->d_name;
        if (entry_name == "." || entry_name == "..") {
            continue;
        }
        std::string full_path = folder_path;
        if (full_path[full_path.length() - 1] != '/') {
            full_path += "/";
        }
        full_path += entry_name;

        struct stat file_stat;
        if (stat(full_path.c_str(), &file_stat) != 0) {
            std::cout << "cat_not_get_file_attribute  " << full_path << std::endl;
            continue;
        }
        if (S_ISDIR(file_stat.st_mode) && recursive) {
            find_color_png_files(full_path, result, match1, match2);
        }
        else if (S_ISREG(file_stat.st_mode)) {
            if (is_target_file(entry_name, match1, match2)) {
                //result.push_back(full_path);
                result.push_back(entry_name);
            }
        }
    }
    closedir(dir);
}
#endif

resultSaveIo::resultSaveIo(algorithmParam& param):algorithm_param_global(param){
    local_img_index_ = 0;
    scale_ = 1.0;
}
resultSaveIo::~resultSaveIo() {

}

void resultSaveIo::thread_write_result_image_data() {
    std::thread(&resultSaveIo::write_result_image_data, this, algorithm_param_global).detach();
}

void savePoseToJson(const std::vector<object_pose>& pose_list, std::string file_path) {
    nlohmann::json j_root;
    //j_root["description"] = "物体位姿数据列表";
    nlohmann::json j_poses = nlohmann::json::array();

    for (int i = 0; i < pose_list.size(); ++i) {
        const auto& pose = pose_list[i];
        // 手动构造，完全避免赋值转换报错
        nlohmann::json j_obj;
        j_obj["id"] = i;
        j_obj["x"] = pose.x;
        j_obj["y"] = pose.y;
        j_obj["z"] = pose.z;
        j_obj["angle_x"] = pose.angle_x;
        j_obj["angle_y"] = pose.angle_y;
        j_obj["angle_z"] = pose.angle_z;
        j_obj["area"] = pose.area;

        j_poses.push_back(j_obj);
    }

    j_root["poses"] = j_poses;

    std::ofstream file(file_path);
    file << j_root.dump(4);
    file.close();
}


void resultSaveIo::write_result_image_data(algorithmParam algorithm_param) {
    std::string write_dir_success = algorithm_param.result_data_path + "/" + algorithm_param.workstation + "/" + algorithm_param.local_time_date + "/";
    std::string write_dir_failed = algorithm_param.result_data_path + "/" + algorithm_param.workstation + "/failed/";;
    
    LOG_INFO("write_path {}",write_dir_success);
    vision::utils::ensureDirectoryExistsRecursive(write_dir_success);
    vision::utils::ensureDirectoryExistsRecursive(write_dir_failed);
    std::string write_name_success = write_dir_success+ algorithm_param.local_time_ms;
    std::string write_name_failed = write_dir_failed+ algorithm_param.local_time_ms;
    if(algorithm_param.cam_data.color.empty() || algorithm_param.cam_data.dep.empty() ||algorithm_param.result_img.empty()){
        return;
    }

    int col = algorithm_param.result_img.cols / 2;
    int row = algorithm_param.result_img.rows / 2;
    cv::Mat result_img;
    cv::resize(algorithm_param.result_img, result_img, cv::Size(col, row));
    cv::Mat tmp_color;
    cv::resize(algorithm_param.cam_data.color, tmp_color, cv::Size(col, row));
    cv::Mat result_h;
    cv::hconcat(tmp_color, result_img, result_h);
    cv::Mat write_color;
    cv::resize(algorithm_param.cam_data.color, write_color, algorithm_param.cam_data.dep.size());

    {
        if (algorithm_param.save_pred)
            cv::imwrite(write_name_success + "_pred.jpg", result_h);
        if (algorithm_param.save_color)
            cv::imwrite(write_name_success + "_color."+algorithm_param.save_image_type, write_color);
        if (algorithm_param.save_depth)
            cv::imwrite(write_name_success + "_depth.png", algorithm_param.cam_data.dep);
        std::string intrinsic_name = write_dir_success + "intrinsic.json";
        std::string hand_eye_matrix_name = write_dir_success + "hand_eye_cali.json";
        if (access(intrinsic_name.c_str(), 0) != 0) {
            nlohmann::json intrinsic_json;
            intrinsic_json["intrinsic"] = algorithm_param.cam_data.intrinsic;
            intrinsic_json["scale"] = algorithm_param.cam_data.scale;
            vision::utils::write_json(intrinsic_name, intrinsic_json);
        }
        if (access(hand_eye_matrix_name.c_str(), 0) != 0) {
            nlohmann::json hand_eye_json;
            hand_eye_json["hand_and_eye"] = algorithm_param.hand_and_eye;
            hand_eye_json["hand_eye_matrix"] = algorithm_param.camera_rt_eye;
            vision::utils::write_json(hand_eye_matrix_name, hand_eye_json);
        }
        //std::string background_img_name = write_dir_success + "background_img.jpg";
        //if(!algorithm_param.back_ground_img.img.empty()){
        //    cv::imwrite(background_img_name, algorithm_param.back_ground_img.img);
        //}
        //savePoseToJson(algorithm_param.multiple_object_pose, write_name_success+"_cube_result.json");
        //savePoseToJson(algorithm_param.multiple_object_pose, write_name_success+"_cube_result.json");
    }
    if (0 != algorithm_param.result_error_data){
        if(algorithm_param.save_pred)
            cv::imwrite(write_name_failed + "_pred.jpg", result_h);
        if(algorithm_param.save_color)
            cv::imwrite(write_name_failed + "_color."+algorithm_param.save_image_type, write_color);
        if(algorithm_param.save_depth)
            cv::imwrite(write_name_failed + "_depth.png", algorithm_param.cam_data.dep);    
        std::string intrinsic_name = write_dir_failed + "intrinsic.json";
        std::string hand_eye_matrix_name = write_dir_failed + "hand_eye_cali.json";
        if (access(intrinsic_name.c_str(), 0) != 0) {
            nlohmann::json intrinsic_json;
            intrinsic_json["intrinsic"] = algorithm_param.cam_data.intrinsic;
            intrinsic_json["scale"] = algorithm_param.cam_data.scale;
            vision::utils::write_json(intrinsic_name, intrinsic_json);
        }
        if (access(hand_eye_matrix_name.c_str(), 0) != 0) {
            nlohmann::json hand_eye_json;
            hand_eye_json["hand_and_eye"] = algorithm_param.hand_and_eye;
            hand_eye_json["hand_eye_matrix"] = algorithm_param.camera_rt_eye;
            vision::utils::write_json(hand_eye_matrix_name, hand_eye_json);
        }

        //savePoseToJson(algorithm_param.multiple_object_pose, write_name_success+"_cube_result.json");

    }
}

void resultSaveIo::write_image_data(cv::Mat color, cv::Mat dep, std::vector<float> intrinsic, float scale) {
    //save_img_number_ = 0;
    //std::string detect_server_name_global = "";
    //std::string save_number_txt = "result_data/" + detect_server_name_global + "_param_image_number.txt";
    //if (_access(save_number_txt.c_str(), 0) != 0) {
    //    LOG_INFO("can_not_find_param_save_image_number.txt");
    //}
    //else {
    //    std::vector<std::string> read_lines;
    //    std::string s;
    //    std::ifstream inf;
    //    inf.open(save_number_txt);
    //    if (inf.is_open()) {
    //        while (getline(inf, s)) {
    //            read_lines.push_back(s);
    //        }
    //        inf.close();
    //        if (read_lines.size() > 0) {
    //            save_img_number_ = std::stod(read_lines[0]);
    //        }
    //    }
    //}
    //std::string sace_path = "result_data/" + detect_server_name_global + "_save_data/";
    //ensureDirectoryExistsRecursive(sace_path);

    //cv::imwrite(sace_path + "/color_" + std::to_string(save_img_number_) + ".png", color);
    //cv::imwrite(sace_path + "/dep_" + std::to_string(save_img_number_) + ".png", dep);
    //std::string para_path = sace_path + "/intrinsic" + std::to_string(save_img_number_) + ".txt";
    //std::ofstream ofs(para_path, std::ios::binary);
    //if (ofs.is_open()) {
    //    size_t size = intrinsic.size();
    //    ofs.write(reinterpret_cast<const char*>(&size), sizeof(size));
    //    if (size > 0) {
    //        ofs.write(reinterpret_cast<const char*>(intrinsic.data()), size * sizeof(float));
    //    }
    //    ofs.write(reinterpret_cast<const char*>(&scale), sizeof(float));
    //    ofs.close();
    //}

    //save_img_number_++;
    //std::ofstream outf;
    //outf.open(save_number_txt);
    //if (outf.is_open()) {
    //    outf << save_img_number_ << std::endl;
    //    outf.close();
    //}
}

void resultSaveIo::write_image_data_xyz(cv::Mat color, cv::Mat dep, std::vector<float> intrinsic, float scale) {
    //std::string detect_server_name_global = "";
    //save_img_number_ = 0;
    //std::string save_number_txt = "result_data/" + detect_server_name_global + "_param_image_number.txt";
    //if (_access(save_number_txt.c_str(), 0) != 0) {
    //    LOG_INFO("can_not_find_param_save_image_number.txt");
    //}
    //else {
    //    std::vector<std::string> read_lines;
    //    std::string s;
    //    std::ifstream inf;
    //    inf.open(save_number_txt);
    //    if (inf.is_open()) {
    //        while (getline(inf, s)) {
    //            read_lines.push_back(s);
    //        }
    //        inf.close();
    //        if (read_lines.size() > 0) {
    //            save_img_number_ = std::stod(read_lines[0]);
    //        }
    //    }
    //}
    //std::string sace_path = "result_data/" + detect_server_name_global + "_save_data/";
    //ensureDirectoryExistsRecursive(sace_path);

    //cv::imwrite(sace_path + "/color_" + std::to_string(save_img_number_) + ".png", color);

    /////////////////////////////////
    //ushort* depdata = (ushort*)dep.data;
    //float dep_z;
    //float dep_x;
    //float dep_y;
    //std::vector<cv::Point3f> points;
    //points.reserve(dep.rows*dep.cols);
    //for (int y = 0; y < dep.rows; ++y)
    //    for (int x = 0; x < dep.cols; ++x) {
    //        dep_z = depdata[y * dep.cols + x]* scale;
    //        if(dep_z>50){
    //            dep_x = (x - intrinsic[2]) * dep_z / intrinsic[0];
    //            dep_y = (y - intrinsic[5]) * dep_z / intrinsic[4];
    //            points.emplace_back(dep_x,dep_y,dep_z);
    //        }
    //    }
    //std::ofstream outf_pt;
    //outf_pt.open(sace_path+"/xyz_point_"+std::to_string(save_img_number_)+".txt");
    //if (outf_pt.is_open()) {
    //    for(int i = 0;i<points.size();++i){
    //         outf_pt << points[i].x <<"  "<<points[i].y<<"  "<<points[i].z << std::endl;
    //    }
    //    outf_pt.close();
    //}

    //save_img_number_++;
    //std::ofstream outf;
    //outf.open(save_number_txt);
    //if (outf.is_open()) {
    //    outf << save_img_number_ << std::endl;
    //    outf.close();
    //}
}

void resultSaveIo::local_img_detect() {
    if (0 != local_img_index_)
        return;
    LOG_INFO("local_img_detect!!!!");
    imgs_path_.clear();
    local_img_index_ = 0;
    std::string match1 = "color";
    std::string match2 = ".png";
    find_color_png_files(local_img_path_, imgs_path_, match1, match2);
    if (imgs_path_.size() < 1) {
        match1 = "color";
        match2 = ".jpg";
        find_color_png_files(local_img_path_, imgs_path_, match1, match2);
    }

    std::vector<std::string> intrinsic_path;
    find_color_png_files(local_img_path_, intrinsic_path, "intrinsic", ".json");
    local_intrinsic_.clear();
    scale_ = 1.0;
    if (intrinsic_path.size() > 0) {
        std::string ins_path = local_img_path_ + "/" + intrinsic_path[0];
        nlohmann::json intrinsic_json;
        vision::utils::read_json(ins_path, intrinsic_json);
        if(intrinsic_json.contains("intrinsic")){
            local_intrinsic_ = intrinsic_json["intrinsic"].get<std::vector<float>>();
        }
        if(intrinsic_json.contains("scale")){
            scale_ = intrinsic_json["scale"].get<float>();
        }
        LOG_INFO("scale_  intrinsic_path  {}  {}", scale_, ins_path);
    }
    else {
        std::vector<std::string> instrinsic_path;
        find_color_png_files(local_img_path_, instrinsic_path, "intrinsic", ".txt");
        local_intrinsic_.clear();
        if (instrinsic_path.size() > 0) {
            std::string ins_path = local_img_path_ + "/" + instrinsic_path[0];
            std::ifstream ifs(ins_path, std::ios::binary);
            if (ifs.is_open()) {
                size_t size = 0;
                ifs.read(reinterpret_cast<char*>(&size), sizeof(size));
                if (size > 0) {
                    local_intrinsic_.resize(size);
                    ifs.read(reinterpret_cast<char*>(local_intrinsic_.data()), size * sizeof(float));
                }
                ifs.read(reinterpret_cast<char*>(&scale_), sizeof(float));
                ifs.close();
            }
        }
    }
}
void resultSaveIo::get_local_img(camera_callback_data& cam_data) {
    local_img_detect();
    if (local_img_index_ < imgs_path_.size() && local_intrinsic_.size() == 9) {
        int p = imgs_path_[local_img_index_].find("color");
        std::string mat_name = imgs_path_[local_img_index_].substr(0, p);
        std::string read_txt_name = local_img_path_ + "/" + "xyz" + mat_name;
        std::string read_dep_name = local_img_path_ + "/" + mat_name + "depth" +".png";
        std::string read_img_name = local_img_path_ + "/" + imgs_path_[local_img_index_];

        cam_data.color = cv::imread(read_img_name, -1);
        cam_data.dep = cv::imread(read_dep_name, -1);
        cam_data.intrinsic = local_intrinsic_;
        cam_data.scale = scale_;
        local_img_index_++;
        if (cam_data.color.empty() || cam_data.dep.empty() || cam_data.intrinsic.size() != 9)
            cam_data.error_data = -2;
        else
            cam_data.error_data = 0;
        LOG_INFO("cam_data_path  {}  {}", read_img_name, read_dep_name);
    }
    else
        cam_data.error_data = -1;
    LOG_INFO("cam_data  {}  {}", cam_data.color.empty(), cam_data.dep.empty());
}
