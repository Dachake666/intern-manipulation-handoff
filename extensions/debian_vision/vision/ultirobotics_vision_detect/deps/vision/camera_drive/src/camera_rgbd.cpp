#include"camera_rgbd.h"
#include <vision_utils/io.h>
cameraRGBD::cameraRGBD(){

}
cameraRGBD::~cameraRGBD(){
    close();
    if (camera_ != nullptr)
    {
      delete camera_;
    }
}
bool cameraRGBD::init(const std::string& config_path,std::string camera_id){
    nlohmann::json cam_config;
    if (!vision::utils::read_json(config_path, cam_config))
    {
      return false;
    }
    std::string camera_mode;
    if(cam_config.contains("model")){
        camera_mode = cam_config["model"];
    }else
        return false;
    if("PS800-E1" ==camera_mode){
        camera_ = new TyCamera();
    }else if("OB335L" ==camera_mode){
        camera_ = new obCamera();
    }else
        return false;
    
    std::string log_path;
    if(cam_config.contains("save_path")){
        log_path = cam_config["save_path"];
    }
    log_path = log_path + "/" + camera_id +".log";
    camera_->set_log(log_path);
    if (!camera_->config_camera(cam_config)) {
        std::cout << "Failed to config TyCamera"<<std::endl;
        return false;
    }
    return true;
}
bool cameraRGBD::open(){
    return camera_->open();
}
bool cameraRGBD::close(){
    return camera_->close();
}
void cameraRGBD::get_cam_data(cv::Mat& color_img,cv::Mat& dep_img,std::vector<float>& intrinsic){
    camera_->get_cam_data(color_img,dep_img,intrinsic);
}
int cameraRGBD::get_cam_state(){
    return camera_->get_cam_state();
}
float cameraRGBD::get_cam_scale(){
    return camera_->get_cam_scale();
}
void cameraRGBD::set_log(std::string path){
    if(camera_!=nullptr){
        camera_->set_log(path);
    }
}
