#pragma once
#ifndef VISION_OBJECT_DETECT_H
#define VISION_OBJECT_DETECT_H
#include "data_type.h"
#include"result_save_io.h"
#include"sam3/sam3_client.h"
#include"image_processing/image_process.h"
#include"yolo/yolov8_seg.h"
#include "point_cloud_processing/point_cloud_object_detector.h"
#include "image_processing/detector_mask_processing.h"
#include"result_post_processor.h"
class visionObjectDetect {
public:
	visionObjectDetect(algorithmParam& param);
	~visionObjectDetect();
	void get_sam3_detect_result(std::vector<yoloseg_detect_result>& sam_result, yoloseg_detect_result& material_box);
	void objecgt_pose_detect(camera_callback_data& cam_data, std::vector<std::string>& text_prompt);
	void read_local_image_data(camera_callback_data& cam_data);
	void write_result_data();
private:
	algorithmParam& algorithm_param_global;
	detectorMaskProcessor mask_process_;
	ImageProcess img_process_;
	yolov8seg yolo_detect_;
	samClient sam3_client_;
	ObjectPointDetect point_detect_;
	resultSaveIo result_save_io_;
	resultPostProcessor reuslt_post_processor_;

	std::shared_ptr<Logger> logger_;
};

#endif // !VISION_OBJECT_DETECT_H
