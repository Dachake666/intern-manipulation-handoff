#include"local_data_test.h"
#include <nlohmann/json.hpp>

#include <filesystem> 
//#include <windows.h>
//#include"io.h"

#ifdef _WIN32
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
    // 1. ����ļ����Ƿ����
    DWORD fileAttr = GetFileAttributesA(dirName.c_str());

    if (fileAttr == INVALID_FILE_ATTRIBUTES) {
        // �ļ��в����ڣ����Դ���
        BOOL createResult = CreateDirectoryA(dirName.c_str(), NULL);
        if (!createResult) {
            std::cerr << "�����ļ��� [" << dirName << "] ʧ�ܣ�������: " << GetLastError() << std::endl;
            return false;
        }
        std::cout << "�ļ��� [" << dirName << "] �Ѵ���" << std::endl;
    }
    else if (fileAttr & FILE_ATTRIBUTE_DIRECTORY) {
        // �ļ����Ѵ���
        std::cout << "�ļ��� [" << dirName << "] �Ѵ���" << std::endl;
    }
    else {
        // ����ͬ���ļ������ļ��У����޷�����
        //std::cerr << "���󣺴���ͬ���ļ� [" << dirName << "]���޷������ļ���" << std::endl;
        return false;
    }

    return true;
}

bool ensureDirectoryExistsRecursive(const std::string& fullDir) {

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
namespace fs = std::filesystem;
localDataRead::localDataRead() {
    const std::string code_file_path = __FILE__;
    const fs::path code_dir = fs::path(code_file_path).parent_path();
    const fs::path local_data_dir = code_dir / "../img";

    local_img_path_ = local_data_dir.string();
     LOG_INFO("local_img_path  {}",local_img_path_);
}
localDataRead::~localDataRead() {

}

void read_local_hand_pose(std::string str,std::vector<float>& data) {
    std::ifstream file(str);
    float num;

    if (file.is_open()) {
        while (file >> num) {
            data.push_back(num);
        }
        file.close();
    }
    else {
        std::cout << "open_file_fail!  "<< str<< std::endl;
        return ;
    }
}

void localDataRead::local_img_detect() {
    if (0 != local_img_index_)
        return;
    
    LOG_INFO("local_img_detect!!!!");
    imgs_path_.clear();
    local_img_index_ = 0;
    std::string match1 = "color";
    std::string match2 = ".png";
    find_color_png_files(local_img_path_, imgs_path_, match1, match2);

    std::vector<std::string> instrinsic_path;
    find_color_png_files(local_img_path_, instrinsic_path, "intrinsic", ".json");
    local_intrinsic_.clear();
    if (instrinsic_path.size() > 0) {
        std::string ins_path = local_img_path_ + "/" + instrinsic_path[0];
        std::ifstream ifs(ins_path);
        if (ifs.is_open()) {
            try {
                nlohmann::json intrinsic_json;
                ifs >> intrinsic_json;
                if (intrinsic_json.contains("intrinsic") && intrinsic_json["intrinsic"].is_array()) {
                    local_intrinsic_ = intrinsic_json["intrinsic"].get<std::vector<float>>();
                }
                if (intrinsic_json.contains("scale")) {
                    scale_ = intrinsic_json["scale"].get<float>();
                }
            }
            catch (const std::exception& e) {
                std::cout << "Failed to parse intrinsic json: " << e.what() << std::endl;
            }
            ifs.close();
        }
    }
}
void localDataRead::get_local_img(camera_callback_data& cam_data,std::vector<float>& hand_pose) {
    local_img_detect();
    if (local_img_index_ < imgs_path_.size() && local_intrinsic_.size() == 9) {
        //int p = imgs_path_[local_img_index_].find("color");
        //std::string mat_name = imgs_path_[local_img_index_].substr(p + 5, -1);
        //std::string read_dep_name = local_img_path_ + "/" + "dep" + mat_name;
        //std::string read_img_name = local_img_path_ + "/" + imgs_path_[local_img_index_];
        //std::string hand_pose_name = local_img_path_ + "/" + "hand_pose" + mat_name +".json";
        int p = imgs_path_[local_img_index_].find("color");
        int p2 = imgs_path_[local_img_index_].find(".png");
        std::string time_name = imgs_path_[local_img_index_].substr(p + 5, p2-p-5);
        std::string read_dep_name = local_img_path_ + "/" + "dep" + time_name + ".png";
        std::string read_img_name = local_img_path_ + "/" + imgs_path_[local_img_index_];
        std::string hand_pose_name = local_img_path_ + "/" + "hand_pose" + time_name +".txt";

        std::vector<float> hand_pose_tp;
        read_local_hand_pose(hand_pose_name, hand_pose_tp);
        cam_data.color = cv::imread(read_img_name, -1);
        cam_data.dep = cv::imread(read_dep_name, -1);
        cam_data.intrinsic = local_intrinsic_;
        cam_data.scale = scale_;
        local_img_index_++;
        if (cam_data.color.empty() || cam_data.dep.empty() || cam_data.intrinsic.size() != 9 ||hand_pose_tp.size()<7)
            cam_data.error_data = -2;
        else
            cam_data.error_data = 0;
        hand_pose = hand_pose_tp;
    }
    else
        cam_data.error_data = -1;
}

