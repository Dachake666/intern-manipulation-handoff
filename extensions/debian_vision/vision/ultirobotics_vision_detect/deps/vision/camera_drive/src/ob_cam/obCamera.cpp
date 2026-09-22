#include "obCamera.h"
#include <fstream>

obCamera::obCamera() {
	is_connect_ = -1;
	is_soft_trigger_ = 0;
    camera_height_ = 800;
    camera_width_ = 1280;
    //set_log("/tmp/obcamera.log");
}

obCamera::~obCamera() {
}

int obCamera::get_cam_state(){
    return is_connect_;
}

void obCamera::cam_reboot() {
	try {
		dev_->reboot();
		std::this_thread::sleep_for(std::chrono::seconds(5));
	}
	catch (ob::Error& e) {
		is_connect_ = -20;
	}
	cam_connect();
}

void obCamera::cam_connect() {
    LOG_INFO("start_obcam_connect !!!! ");
	try {
		while (1) {
			ob::Context ctx;
			std::shared_ptr<ob::DeviceList> devList = ctx.queryDeviceList();
			int cam_count = devList->getCount();
            LOG_INFO("obcam_count !!!! {}",cam_count);
			if (cam_count < 1) {
				is_connect_ = -2;
				std::this_thread::sleep_for(std::chrono::seconds(1));
				continue;
			}
			dev_ = devList->getDevice(0);
			/*
			OBMultiDeviceSyncConfig dev_config = dev_->getMultiDeviceSyncConfig();
            if (is_soft_trigger_){
                LOG_INFO("obcam_trigger_OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING !!!! ");
				dev_config.syncMode = OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING;
            }
            else{
                LOG_INFO("obcam_trigger_OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN !!!!");
				dev_config.syncMode = OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN;
            }
			dev_->setMultiDeviceSyncConfig(dev_config);
			//*/
			pipe_ = std::make_shared<ob::Pipeline >(dev_);
			auto config = std::make_shared<ob::Config>();

			config->enableVideoStream(OB_STREAM_DEPTH, camera_width_, camera_height_, 5, OB_FORMAT_Y16);//OB_FPS_ANY
			config->enableVideoStream(OB_STREAM_COLOR, camera_width_, camera_height_, 5, OB_FORMAT_BGR);//OB_FORMAT_BGR

			config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

			pipe_->enableFrameSync();

			pipe_->start(config);
            LOG_INFO(" obcam_pipe_start!!!!");

			if (is_soft_trigger_) {
				int count = 0;
				std::shared_ptr<ob::FrameSet> frameset = nullptr;
				while (true) {
					dev_->triggerCapture();
					count++;
					if (count > 30) {
						std::thread{ std::bind(&obCamera::cam_reboot, this) }.detach();
						is_connect_ = -3;
						return;
					}
					frameset = pipe_->waitForFrameset(1000);
					if (frameset) {
						auto colorFrame = frameset->getFrame(OB_FRAME_COLOR);
						auto depthFrame = frameset->getFrame(OB_FRAME_DEPTH);
						if (colorFrame && depthFrame) {
							break;
						}
					}
				}
			}
            LOG_INFO("obcam_connect_success !!!!");
			is_connect_ = 0;
			break;
		}
	}
	catch (ob::Error& e) {
		is_connect_ = -20;
		std::cout << e.getMessage() << std::endl;
	}

}

void obCamera::get_cam_img(cv::Mat& color_img, cv::Mat& dep_img, std::vector<float>& intrinsic) {
    if (0 != is_connect_){
        LOG_INFO(" obcam_is_connect_error_get_cam_img!!!!");
		return;
    }
	int count = 0;
	std::shared_ptr<ob::FrameSet> frameset = nullptr;
	while (true) {
		count++;
		if (count > 9) {
			cam_reboot();
			is_connect_ = -3;
			break;
		}
		if (is_soft_trigger_) {
			try {
				dev_->triggerCapture();
			}
			catch (ob::Error& e) {
                LOG_INFO(" get_obcam_trigger_error!");
				is_connect_ = -20;
				continue;
			}
		}
		try {
			frameset = pipe_->waitForFrameset(1000);
		}
		catch (ob::Error& e) {
            LOG_INFO(" get_obcam_error!");
			is_connect_ = -20;
			continue;
		}
		if (frameset) {
			auto colorFrame = frameset->getFrame(OB_FRAME_COLOR);
			auto depthFrame = frameset->getFrame(OB_FRAME_DEPTH);
			if (colorFrame && depthFrame) {
				auto videoFrame = colorFrame->as<const ob::VideoFrame>();
				uint32_t color_height = videoFrame->getHeight();
				uint32_t color_width = videoFrame->getWidth();
				color_img = cv::Mat(color_height, color_width, CV_8UC3, videoFrame->getData());
				auto colorProfile = colorFrame->getStreamProfile();
				ob_camera_intrinsic colorIntrinsic = colorProfile->as<ob::VideoStreamProfile>()->getIntrinsic();

				auto pointCloudFilter = std::make_shared<ob::PointCloudFilter>();
				auto alignFilter = std::make_shared<ob::Align>(OB_STREAM_COLOR);

				auto aliggnFrameset = alignFilter->process(frameset);

				pointCloudFilter->setCreatePointFormat(OB_FORMAT_RGB_POINT);
				auto result = pointCloudFilter->process(aliggnFrameset);

				uint32_t      pointsSize = result->dataSize() / sizeof(OBColorPoint);
				OBColorPoint* point = (OBColorPoint*)result->data();

				int pt_width = colorFrame->as<ob::VideoFrame>()->width();
				int pt_height = colorFrame->as<ob::VideoFrame>()->height();

				if (pt_width * pt_height != pointsSize || pt_width != color_width || pt_height != color_height) {
					is_connect_ = -5;
					break;
				}
                dep_img = cv::Mat::zeros(pt_height, pt_width, CV_16UC1);
				ushort* depdata = (ushort*)dep_img.data;

				for (int y = 0; y < dep_img.rows; ++y)
					for (int x = 0; x < dep_img.cols; ++x) {
						depdata[y * dep_img.cols + x] = point->z;
						point++;
					}
				if (intrinsic_.size() > 0) {
					intrinsic = intrinsic_;
				}
				else {
					intrinsic_.resize(9);
					intrinsic_[0] = colorIntrinsic.fx;
					intrinsic_[1] = 0;
					intrinsic_[2] = colorIntrinsic.cx;
					intrinsic_[3] = 0;
					intrinsic_[4] = colorIntrinsic.fy;
					intrinsic_[5] = colorIntrinsic.cy;
					intrinsic_[6] = 0;
					intrinsic_[7] = 0;
					intrinsic_[8] = 1;
					intrinsic = intrinsic_;
				}
				break;
			}
		}
	}
}


bool obCamera::config_camera(nlohmann::json& config) {
    LOG_INFO("obcam_config_camera !");
    is_soft_trigger_ = config["soft_trigger"];
    camera_height_ = config["camera_height"];
    camera_width_ = config["camera_width"];
    LOG_INFO("obcam_config_camera trigger_height_width ! {}  {}  {}",is_soft_trigger_,camera_height_,camera_width_);
	return 1;
}
bool obCamera::open() {
    std::thread{ std::bind(&obCamera::cam_connect, this) }.detach();
    std::this_thread::sleep_for(std::chrono::seconds(5));
	return 1;
}
bool obCamera::close() {
    if (pipe_) pipe_->stop();
	return 1;
}
void obCamera::get_cam_data(cv::Mat& color_img, cv::Mat& dep_img, std::vector<float>& intrinsic) {
	cv::Mat xyz;
    LOG_INFO(" get_cam_data_obcamera!");
	get_cam_img(color_img, dep_img, intrinsic);
	return;
}
float obCamera::get_cam_scale(){
    return 1.0;
}
