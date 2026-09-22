#include"tyCam.h"
#include "TYImageProc.h"
#include <fstream>
#include <iostream>
#include <chrono>
//#include "io.h"

int undistort_img(const TY_CAMERA_CALIB_INFO& color_calib,cv::Mat& color,cv::Mat& undistort_color) {
	int32_t  image_size;
	TYPixFmt color_fmt;
	if (color.type() == CV_16U) {
		image_size = color.size().area() * 2;
		color_fmt = TYPixelFormatMono16;
	}
	else {
		image_size = color.size().area() * 3;
		color_fmt = TYPixelFormatRGB8;
	}
	// do undistortion

	if (color_fmt == TYPixelFormatMono16)
		undistort_color = cv::Mat(color.size(), CV_16U);
	else
		undistort_color = cv::Mat(color.size(), CV_8UC3);

	TY_IMAGE_DATA src;
	src.width = color.cols;
	src.height = color.rows;
	src.size = image_size;
	src.pixelFormat = color_fmt;
	src.buffer = color.data;

	TY_IMAGE_DATA dst;
	dst.width = color.cols;
	dst.height = color.rows;
	dst.size = image_size;
	dst.pixelFormat = color_fmt;
	dst.buffer = undistort_color.data;
	int ret = TYUndistortImage(&color_calib, &src, NULL, &dst);
	return ret;
}

static void doRegister(const TY_CAMERA_CALIB_INFO& depth_calib
    , const TY_CAMERA_CALIB_INFO& color_calib
    , const cv::Mat& depth
    , const float f_scale_unit
    , const cv::Mat& color
    , bool needUndistort
    , cv::Mat& undistort_color
    , cv::Mat& out
    , bool map_depth_to_color
)
{
    int32_t         image_size;
    TYPixFmt color_fmt;
    if (color.type() == CV_16U) {
        image_size = color.size().area() * 2;
        color_fmt = TYPixelFormatMono16;
    }
    else {
        image_size = color.size().area() * 3;
        color_fmt = TYPixelFormatRGB8;
    }
    // do undistortion
    if (needUndistort) {
        if (color_fmt == TYPixelFormatMono16)
            undistort_color = cv::Mat(color.size(), CV_16U);
        else
            undistort_color = cv::Mat(color.size(), CV_8UC3);

        TY_IMAGE_DATA src;
        src.width = color.cols;
        src.height = color.rows;
        src.size = image_size;
        src.pixelFormat = color_fmt;
        src.buffer = color.data;

        TY_IMAGE_DATA dst;
        dst.width = color.cols;
        dst.height = color.rows;
        dst.size = image_size;
        dst.pixelFormat = color_fmt;
        dst.buffer = undistort_color.data;
        ASSERT_OK(TYUndistortImage(&color_calib, &src, NULL, &dst));
    }
    else {
        undistort_color = color;
    }

    // do register
    if (map_depth_to_color) {
        int outW = depth.cols;
        int outH = depth.cols * undistort_color.rows / undistort_color.cols;
        out = cv::Mat::zeros(cv::Size(outW, outH), CV_16U);
        ASSERT_OK(
            TYMapDepthImageToColorCoordinate(
                &depth_calib,
                depth.cols, depth.rows, depth.ptr<uint16_t>(),
                &color_calib,
                out.cols, out.rows, out.ptr<uint16_t>(), f_scale_unit
            )
        );
        cv::Mat temp;
        cv::resize(out, temp, undistort_color.size(), 0, 0, cv::INTER_NEAREST);
        out = temp;
    }
    else {
        if (color_fmt == TYPixelFormatMono16)
        {
            out = cv::Mat::zeros(depth.size(), CV_16U);
            ASSERT_OK(
                TYMapMono16ImageToDepthCoordinate(
                    &depth_calib,
                    depth.cols, depth.rows, depth.ptr<uint16_t>(),
                    &color_calib,
                    undistort_color.cols, undistort_color.rows, undistort_color.ptr<uint16_t>(),
                    out.ptr<uint16_t>(), f_scale_unit
                )
            );
        }
        else {
            out = cv::Mat::zeros(depth.size(), CV_8UC3);
            ASSERT_OK(
                TYMapRGBImageToDepthCoordinate(
                    &depth_calib,
                    depth.cols, depth.rows, depth.ptr<uint16_t>(),
                    &color_calib,
                    undistort_color.cols, undistort_color.rows, undistort_color.ptr<uint8_t>(),
                    out.ptr<uint8_t>(), f_scale_unit
                )
            );
        }
    }
}


void eventCallback(TY_EVENT_INFO* event_info, void* userdata)
{
    std::cout<<"camera_eventCallback_is_offline"<<std::endl;
    if (event_info->eventId == TY_EVENT_DEVICE_OFFLINE) {
        std::cout<<"camera_offline"<<std::endl;
        if (userdata) {
            *((bool*)userdata) = true;
        }
    }
    else if (event_info->eventId == TY_EVENT_LICENSE_ERROR) {
        std::cout<<"camera_license_error"<<std::endl;
    }
}

bool set_resolution(TY_DEV_HANDLE device, TY_COMPONENT_ID comopent_id, const std::pair<int, int>& resolution) {
    int ret = TY_STATUS_OK;
    uint32_t n = 0;
    ret = TYGetEnumEntryCount(device, comopent_id, TY_ENUM_IMAGE_MODE, &n);
    if (ret != TY_STATUS_OK)
    {
        return false;
    }

    std::vector<TY_ENUM_ENTRY> image_mode_list(n);
    ret = TYGetEnumEntryInfo(device, comopent_id, TY_ENUM_IMAGE_MODE, &image_mode_list[0], n, &n);
    if (ret != TY_STATUS_OK)
    {
        return false;
    }

    for (const auto& mode : image_mode_list)
    {
        if (TYImageWidth(mode.value) == resolution.first ||
            TYImageHeight(mode.value) == resolution.second)
        {
            int ret = TYSetEnum(device, comopent_id, TY_ENUM_IMAGE_MODE, mode.value);
            return (ret == TY_STATUS_OK || ret == TY_STATUS_NOT_PERMITTED);
        }
    }
    return false;
}

bool TyCamera::get_img(CamInfo& cams, cv::Mat& img,cv::Mat& dep, std::vector<float>& intrinsic)
{
    int count = 0;
    LOG_INFO("tycam_start_get_img");
    time_t start, end;
    start = time(NULL);
    while (1)
    {
        count++;
        end = time(NULL);
        if (difftime(end, start) > 10)
        {
            connet_state_ = -3;
            LOG_INFO("tycam_get_img_out_time!!!!!!!!!!!!  {}", count);
            close();
            std::thread{ std::bind(&TyCamera::Init_camera, this) }.detach();
            std::this_thread::sleep_for(std::chrono::seconds(5));
            break;
        }
        cv::Mat vc_color;
        cv::Mat depth, irl, irr;
        if (0 != TYSendSoftTrigger(cams.hDevice))
        {
            LOG_INFO("tycam_get_img_soft_trigger_fail {}", count);
            continue;
        }
        int err = TYFetchFrame(cams.hDevice, &cams.frame, 20000);
        if (err != TY_STATUS_OK) {
            LOG_INFO("tycam_get_img_TYFetchFrame_fail  {}", count);
            continue;
        }
        if (0 != parseFrame(cams.frame, &depth, &irl, &irr, &vc_color)){
            LOG_INFO("tycam_get_img_parseFrame_fail  {}", count);
            continue;
        }

        cv::Mat color_temp;
        if(vc_color.cols == depth.cols&&vc_color.rows == depth.rows)
            color_temp = vc_color.clone();
        else
            cv::resize(vc_color,color_temp,cv::Size(depth.cols,depth.rows));

        cv::Mat undistort_color, out;
        //doRegister(cams.depth_calib, cams.color_calib, depth, cams.scale_unit, vc_color, true, undistort_color, out, true);
        doRegister(cams.depth_calib, cams.color_calib, depth, cams.scale_unit, color_temp, true, undistort_color, out, true);

        err = undistort_img(cams.color_calib ,vc_color ,undistort_color);
        if (err != TY_STATUS_OK) {
            LOG_INFO("undistort_img_fail  {}", count);
            continue;
        }

        img = undistort_color.clone();
        dep = out.clone();
        intrinsic.clear();
        //float ratio_x = 1.0*cam_param_.color_resolution_width_ /cams.color_calib.intrinsicWidth;
        //float ratio_y = 1.0*cam_param_.color_resolution_height_ / cams.color_calib.intrinsicHeight;
        float ratio_x = 1.0*dep.cols /cams.color_calib.intrinsicWidth;
        float ratio_y = 1.0*dep.rows/ cams.color_calib.intrinsicHeight;
        for (int i = 0; i < 9; ++i) {
            //intrinsic.push_back(cams.depth_calib.intrinsic.data[i]);
            intrinsic.push_back(cams.color_calib.intrinsic.data[i]);
        }
        intrinsic[0] *= ratio_x;
        intrinsic[2] *= ratio_x;
        intrinsic[4] *= ratio_y;
        intrinsic[5] *= ratio_y;

        // ������������bufferѭ��ʹ�õĸ�����ÿ��buffer��dequeue�û�ȡ��֡�������֡���������У�����ְѵ�ǰ��bufferѹ������У��?��ʹ�ã�
        // ���ڶ������ԣ����׳��У���β���У����Ի���ѭ��ʹ�õ����ӡ�
        TYEnqueueBuffer(cams.hDevice, cams.frame.userBuffer, cams.frame.bufferSize);

        //img = vc_color.clone();
        LOG_INFO("tycam_get_img_success��{}", count);
        break;
    }
    return true;
}

TyCamera::TyCamera()
{
    cam_.hDevice = NULL;
    connet_state_ = -1;
    cam_.scale_unit = 0.25;
    is_offline_ = false ;
}

bool TyCamera::open() {
    std::thread{ std::bind(&TyCamera::Init_camera, this) }.detach();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    return 1;
}

void TyCamera::Init_camera()
{
    bool first = true;
    int ty_status = 0;
    if (cam_.hDevice!=NULL) {
        TYCloseDevice(cam_.hDevice);
        TYDeinitLib();
    }
    int count = 0;
    LOG_INFO("tycam_start_init!");
    uint64_t time_log_last = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    while (1){
        count++;
        std::string ID, IP;// = "192.168.0.225";
        if ((ty_status = TYInitLib()) != 0) {
            //connet_state_ = -2;
            uint64_t time_log =  std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            if(time_log -time_log_last>1000 || first){
                time_log_last = time_log;
                LOG_INFO("tycam_init_TYInitLib_fail!  {}", ty_status);
                first = false;
            }
            //continue;
        }
        TY_VERSION_INFO ver;
        if ((ty_status = TYLibVersion(&ver)) != 0){
            connet_state_ = -2;
            uint64_t time_log =  std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            if(time_log -time_log_last>1000 || first){
                time_log_last = time_log;
                LOG_INFO("tycam_init_TYLibVersion_fail!  {}", ty_status);
                first = false;
            }
            TYDeinitLib();
            continue;
        }
        std::vector<TY_DEVICE_BASE_INFO> selected;

        if ((ty_status = selectDevice(TY_INTERFACE_ALL, ID, IP, 20, selected)) != 0){
            connet_state_ = -4;
            LOG_INFO("tycam_init_selectDevice_fail!  {}", ty_status);
            TYDeinitLib();
            continue;
        }
        if (selected.size() <1){
            connet_state_ = -5;
            LOG_INFO("tycam_init_camera_numbers<1!  {}", count);
            TYDeinitLib();
            continue;
        }
        bool flag_ct = 0;
        bool is_found_true_ip = 0;
        for (int i = 0; i < selected.size(); ++i) {
            if (cam_param_.ip_ != selected[i].netInfo.ip)
                continue;
            is_found_true_ip = 1;
            cam_.IP = selected[i].netInfo.ip;
            if ((ty_status=TYOpenInterface(selected[i].iface.id, &(cam_.hIface))) != 0){
                connet_state_ = -7;
                LOG_INFO("tycam_init_TYOpenInterface_fail!  {}", ty_status);
                TYDeinitLib();
                flag_ct = 1;
                break;
            }
            if ((ty_status = TYOpenDevice(cam_.hIface, selected[i].id, &(cam_.hDevice))) != 0){
                connet_state_ = -7;
                LOG_INFO("tycam_init_TYOpenDevice_fail!   {}", ty_status);
                TYCloseDevice(cam_.hDevice);
                TYDeinitLib();
                flag_ct = 1;
                break;
            }

            uint32_t allComps;
            if (0 != (ty_status = TYGetComponentIDs(cam_.hDevice, &allComps))){
                connet_state_ = -7;
                LOG_INFO("tycam_init_TYGetComponentIDs_fail!  {}", ty_status);
                TYCloseDevice(cam_.hDevice);
                TYDeinitLib();
                flag_ct = 1;
                break;
            }
    //			if (0 != TYEnableComponents(cams[i].hDevice, TY_COMPONENT_RGB_CAM | TY_COMPONENT_DEPTH_CAM | TY_COMPONENT_IR_CAM_LEFT | TY_COMPONENT_IR_CAM_RIGHT))
            if (0 != (ty_status = TYEnableComponents(cam_.hDevice, TY_COMPONENT_RGB_CAM | TY_COMPONENT_DEPTH_CAM))){
                connet_state_ = -8;
                LOG_INFO("tycam_init_TYEnableComponents_fail!   {}", ty_status);
                TYCloseDevice(cam_.hDevice);
                TYDeinitLib();
                flag_ct = 1;
                break;
            }
            if (1 != set_camera_param(cam_.hDevice)) {
                connet_state_ = -8;
                LOG_INFO("tycam_set_camera_param_fail!   {}", count);
                TYCloseDevice(cam_.hDevice);
                TYDeinitLib();
                flag_ct = 1;
                break;
            }
            connet_state_ = -9;
            //if (0 != TYSetEnum(cam_.hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, TY_IMAGE_MODE_DEPTH16_1280x960)) {
            //	LOG_INFO("tycam_init_TYSetEnum_fail_dep!   {}", count);
            //	TYCloseDevice(cam_.hDevice);
            //	TYDeinitLib();
            //	flag_ct = 1;
            //	break;
            //}
            //if (0 != TYSetEnum(cam_.hDevice, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, TY_IMAGE_MODE_YUYV_1280x960)) {
            //	LOG_INFO("tycam_init_TYSetEnum_fail_rgb!   {}", count);
            //	TYCloseDevice(cam_.hDevice);
            //	TYDeinitLib();
            //	flag_ct = 1;
            //	break;
            //}

            uint32_t frameSize;
            if ((ty_status = TYGetFrameBufferSize(cam_.hDevice, &frameSize)) != 0){
                LOG_INFO("tycam_init_TYGetFrameBufferSize_fail!   {}", ty_status);
                TYCloseDevice(cam_.hDevice);
                TYDeinitLib();
                flag_ct = 1;
                break;
            }
            cam_.frameBuffer[0].resize(frameSize);
            cam_.frameBuffer[1].resize(frameSize);

            if ((ty_status=TYEnqueueBuffer(cam_.hDevice, cam_.frameBuffer[0].data(), frameSize)) != 0){
                LOG_INFO("tycam_init_TYEnqueueBuffer0_fail!   {}", ty_status);
                TYCloseDevice(cam_.hDevice);
                TYDeinitLib();
                flag_ct = 1;
                break;
            }
            if ((ty_status = TYEnqueueBuffer(cam_.hDevice, cam_.frameBuffer[1].data(), frameSize)) != 0){
                LOG_INFO("tycam_init_TYEnqueueBuffer1_fail!  {}", ty_status);
                TYCloseDevice(cam_.hDevice);
                TYDeinitLib();
                flag_ct = 1;
                break;
            }
            float scale_unit = 1.;
            if (0!= (ty_status = TYGetFloat(cam_.hDevice, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &scale_unit))) {
                LOG_INFO("tycam_init_TYGetFloat_fail!  {}", ty_status);
                TYCloseDevice(cam_.hDevice);
                TYDeinitLib();
                flag_ct = 1;
                break;
            }
            cam_.scale_unit = scale_unit;

            if (0 != (ty_status = TYGetStruct(cam_.hDevice, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_CALIB_DATA
                , &(cam_.depth_calib), sizeof(cam_.depth_calib)))) {
                LOG_INFO("tycam_init_TYGetStruct_dep_fail!  {}", ty_status);
                TYCloseDevice(cam_.hDevice);
                TYDeinitLib();
                flag_ct = 1;
                break;
            }
            if (0 != (ty_status = TYGetStruct(cam_.hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_CALIB_DATA
                , &cam_.color_calib, sizeof(cam_.color_calib)))) {
                LOG_INFO("tycam_init_TYGetStruct_color_fail!  {}", ty_status);
                TYCloseDevice(cam_.hDevice);
                TYDeinitLib();
                flag_ct = 1;
                break;
            }
            is_offline_ = false;
            if (0 != (ty_status = TYRegisterEventCallback(cam_.hDevice, eventCallback, &is_offline_))){
                LOG_INFO("tycam_init_TYRegisterEventCallback_fail!   {}", ty_status);
                TYCloseDevice(cam_.hDevice);
                TYDeinitLib();
                flag_ct = 1;
                break;
            }
            TY_TRIGGER_PARAM trigger;
            trigger.mode = TY_TRIGGER_MODE_M_SIG;//TY_TRIGGER_MODE_SLAVE;
            if (0 != (ty_status = TYSetStruct(cam_.hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &trigger, sizeof(trigger)))){
                LOG_INFO("tycam_init_TYSetStruct_fail!   {}", ty_status);
                TYCloseDevice(cam_.hDevice);
                TYDeinitLib();
                flag_ct = 1;
                break;
            }

            int32_t resend = 1;
            if (resend) {
                bool hasResend;
                if (0 != (ty_status = TYHasFeature(cam_.hDevice, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, &hasResend))) {
                    LOG_INFO("tycam_init_TYHasFeature_fail!   {}", ty_status);
                    TYCloseDevice(cam_.hDevice);
                    TYDeinitLib();
                    flag_ct = 1;
                    break;
                }
                if (hasResend) {
                    LOG_INFO("tycam_init_Open resend!");
                    if ((ty_status = TYSetBool(cam_.hDevice, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, true)) !=0) {
                        LOG_INFO("tycam_init_TYSetBool_fail!   {}", ty_status);
                        TYCloseDevice(cam_.hDevice);
                        TYDeinitLib();
                        flag_ct = 1;
                        break;
                    }
                }
                else {
                    LOG_INFO("tycam_init_ Not_support_feature_TY_BOOL_GVSP_RESEND!  {}", count);
                }
            }

            if ((ty_status = TYStartCapture(cam_.hDevice)) != 0)
            {
                LOG_INFO("tycam_init_TYStartCapture_fail!  {}", ty_status);
                TYCloseDevice(cam_.hDevice);
                TYDeinitLib();
                flag_ct = 1;
                break;
            }
            break;
        }
        if(0==is_found_true_ip){
            connet_state_ = -6;
            continue;
        }
        if (flag_ct)
        {
            continue;
        }
        connet_state_ = 0;
        LOG_INFO("tycam_init_success!  {}", count);
        break;
    }
}

void TyCamera::get_cam_data(cv::Mat& color_img, cv::Mat& dep_img, std::vector<float>& intrinsic) {
    if (is_offline_) {
        LOG_INFO("检测到相机断线，开始自动重连...");     
        close();
        connet_state_ = -3;
        Init_camera();
        LOG_INFO("相机重连成功！");
    }
    if (0 != connet_state_)
        return;
    get_img(cam_, color_img, dep_img, intrinsic);
    if (!color_img.empty() && !dep_img.empty() && intrinsic.size()==9) {
        LOG_INFO("get_color_dep_xyz_success_! {}  {}  {}",color_img.empty(),dep_img.empty(),intrinsic.size());
    }
    else {
        LOG_INFO("get_color_dep_xyz_fail! {}  {}  {}",color_img.empty(),dep_img.empty(),intrinsic.size());
    }
    return;
}

TyCamera::~TyCamera()
{
    TYStopCapture(cam_.hDevice);
    TYCloseDevice(cam_.hDevice);

    TYDeinitLib();
}


bool TyCamera::close()
{
    TYStopCapture(cam_.hDevice);
    //TYCloseDevice(cam_.hDevice);
    //TYDeinitLib();
    return true;
}

bool TyCamera::config_camera(nlohmann::json& config) {
    cam_param_.ip_ = config["camera_ip"];
    cam_param_.color_exposure_ = config["color"]["exposure"];
    cam_param_.color_resolution_width_ = config["color"]["resolution"][0];
    cam_param_.color_resolution_height_ = config["color"]["resolution"][1];
    cam_param_.color_analog_gain_ = config["color"]["analog_gain"];
    cam_param_.dep_resolution_width_ = config["depth"]["resolution"][0];
    cam_param_.dep_resolution_height_ = config["depth"]["resolution"][1];
    cam_param_.left_ir_exposure_ = config["left_ir"]["exposure"];
    cam_param_.left_gain_ = config["left_ir"]["gain"];
    cam_param_.left_analog_gain_ = config["left_ir"]["analog_gain"];
    cam_param_.right_ir_exposure_ = config["right_ir"]["exposure"];
    cam_param_.right_gain_ = config["right_ir"]["gain"];
    cam_param_.right_analog_gain_ = config["right_ir"]["analog_gain"];
    cam_param_.power_level_ = config["power_level"];
    return true;
}

bool TyCamera::set_camera_param(TY_DEV_HANDLE device) {
    int ret = TY_STATUS_OK;
    /*
    bool resend_ = true; 
    ret = TYSetBool(device, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, resend_);
    if (ret != TY_STATUS_OK)
    {
      LOG_ERROR("set resend failed");
      return false;
    }
    //*/
    if (!set_resolution(device,TY_COMPONENT_DEPTH_CAM, std::pair<int, int>(cam_param_.dep_resolution_width_, cam_param_.dep_resolution_height_))){
        LOG_ERROR("set depth_resolution failed");
        return false;
    }

    if (!set_resolution(device,TY_COMPONENT_RGB_CAM, std::pair<int, int>(cam_param_.color_resolution_width_, cam_param_.color_resolution_height_))){
        LOG_ERROR("set rgb_resolution failed");
        return false;
    }

    ret = TYSetInt(device, TY_COMPONENT_RGB_CAM, TY_INT_ANALOG_GAIN, cam_param_.color_analog_gain_);
    if (ret != TY_STATUS_OK)
    {
        LOG_ERROR("set color_analog_gain failed");
        return false;
    }
    ////////////
    ret = TYSetInt(device, TY_COMPONENT_LASER, TY_INT_LASER_POWER, cam_param_.power_level_);
    if (ret != TY_STATUS_OK)
    {
        LOG_ERROR("set laser_power failed");
        return false;
    }
    //////////////
    ret = TYSetInt(device, TY_COMPONENT_RGB_CAM, TY_INT_EXPOSURE_TIME, cam_param_.color_exposure_);
    if (ret != TY_STATUS_OK)
    {
        LOG_ERROR("set color_exposure failed");
        return false;
    }

    ret =TYSetInt(device, TY_COMPONENT_IR_CAM_LEFT, TY_INT_GAIN, cam_param_.left_gain_);
    if (ret != TY_STATUS_OK)
    {
        LOG_ERROR("set left_ir_gain failed");
        return false;
    }
    ret = TYSetInt(
        device,
        TY_COMPONENT_IR_CAM_LEFT,
        TY_INT_ANALOG_GAIN,
        cam_param_.left_analog_gain_);
    if (ret != TY_STATUS_OK)
    {
        LOG_ERROR("set left_ir_analog_gain failed");
        return false;
    }
    //ret = TYSetInt(
    //	device,
    //	TY_COMPONENT_IR_CAM_LEFT,
    //	TY_INT_EXPOSURE_TIME,
    //	cam_param_.left_ir_exposure_);
    //if (ret != TY_STATUS_OK)
    //{
    //	LOG_ERROR("set left_ir_exposure failed");
    //	return false;
    //}

    ret = TYSetInt(
        device, TY_COMPONENT_IR_CAM_RIGHT, TY_INT_GAIN, cam_param_.right_gain_);
    if (ret != TY_STATUS_OK)
    {
        LOG_ERROR("set right_ir_gain failed");
        return false;
    }
    ret = TYSetInt(
        device,
        TY_COMPONENT_IR_CAM_RIGHT,
        TY_INT_ANALOG_GAIN,
        cam_param_.right_analog_gain_);
    if (ret != TY_STATUS_OK)
    {
        LOG_ERROR("set right_ir_analog_gain failed");
        return false;
    }
    //ret = TYSetInt(
    //	device,
    //	TY_COMPONENT_IR_CAM_RIGHT,
    //	TY_INT_EXPOSURE_TIME,
    //	cam_param_.right_ir_exposure_);
    //if (ret != TY_STATUS_OK)
    //{
    //	LOG_ERROR("set right_ir_exposure failed");
    //	return false;
    //}
    return true;
}


int TyCamera::get_cam_state() {
    return connet_state_;
}

float TyCamera::get_cam_scale(){
    return cam_.scale_unit;
}

//void TyCamera::set_log(std::shared_ptr<Logger> logger){
//    logger_ = logger;
//}





