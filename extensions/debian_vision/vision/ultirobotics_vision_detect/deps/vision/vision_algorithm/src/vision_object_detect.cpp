#include"vision_object_detect.h"
#include "geometry_utils.h"
visionObjectDetect::visionObjectDetect(algorithmParam& param):algorithm_param_global(param),mask_process_(param),
                    img_process_(param), point_detect_(param), result_save_io_(param){
	sam3_client_.creat_samClient(param.sam3_server_ip);
	std::string log_name = param.result_data_path + "/" + param.workstation + "/vision_detect.log";
	logger_ = std::make_shared<Logger>(log_name);
	mask_process_.logger_ = logger_;
	img_process_.logger_ = logger_;
	point_detect_.logger_ = logger_;
	result_save_io_.logger_ = logger_;
	yolo_detect_.init(param.model_path);
}
visionObjectDetect::~visionObjectDetect() {

}

void visionObjectDetect::read_local_image_data(camera_callback_data& cam_data) {
	result_save_io_.get_local_img(cam_data);
}

void visionObjectDetect::get_sam3_detect_result(std::vector<yoloseg_detect_result>& sam_result, yoloseg_detect_result& material_box) {
	std::vector<sam3_result> sam_data;
	int error_data;
	sam3_client_.get_sam3_result_data(sam_data, error_data);
	LOG_WARN("sam3_result_size!! {}", sam_data.size());
	sam_result.resize(sam_data.size());
	//* 
    for (int i = 0; i < sam_data.size(); ++i) {
		sam_result[i].box = sam_data[i].box;
		sam_result[i].class_id = sam_data[i].cls_id;
		sam_result[i].confidence = sam_data[i].confidence;
		sam_result[i].part_mask = sam_data[i].part_mask.clone();
	}
	   //*/
	/*
	float ratio_x = 1.0 * algorithm_param_global.cam_data.color.cols / algorithm_param_global.cam_data.dep.cols;
	float ratio_y = 1.0 * algorithm_param_global.cam_data.color.rows / algorithm_param_global.cam_data.dep.rows;
	cv::Rect temp;
	for (int i = 0; i < sam_data.size(); ++i) {
		temp.x = (sam_data[i].box.x + material_box.box.x * ratio_x) / ratio_x;
		temp.y = (sam_data[i].box.y + material_box.box.y * ratio_y) / ratio_y;
		temp.width = sam_data[i].box.width / ratio_x;
		temp.height = sam_data[i].box.height / ratio_y;
		sam_result[i].box = temp;
		//detect_result[i].box = sam_data[i].box;
		sam_result[i].class_id = 1;
		sam_result[i].confidence = sam_data[i].confidence;
		cv::resize(sam_data[i].part_mask, sam_result[i].part_mask, cv::Size(sam_data[i].part_mask.cols / ratio_x, sam_data[i].part_mask.rows / ratio_y));
		//detect_result[i].part_mask = sam_data[i].part_mask.clone();
	}
	//**/

}

std::vector<std::vector<double>> transform_point(
    const float* T,
    const std::vector<std::vector<double>>& p_cam)
{
    // 初始化 4x4 结果矩阵
    std::vector<std::vector<double>> res(4, std::vector<double>(4, 0.0));

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            // 矩阵乘法核心公式：第i行 × 第j列 求和
            res[i][j] = T[i*4 + 0] * p_cam[0][j] +
                        T[i*4 + 1] * p_cam[1][j] +
                        T[i*4 + 2] * p_cam[2][j] +
                        T[i*4 + 3] * p_cam[3][j];
        }
    }

    return res;
}

void get_rt_inv(std::vector<std::vector<double>>& rt, std::vector<double>& rt_inv) {
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
bool get_transform_matrix_eye_on_hand(algorithmParam& algorithm_param_global) {
	cv::Mat transMat = poseVectorToTransformMatrix_q(algorithm_param_global.hand_pose);
	if(transMat.empty()&&transMat.rows != 4&&transMat.cols != 4){
		return false;
	}
	float* tran_data = (float*)transMat.data;
	std::vector<std::vector<double>> temp_cam_rt(4);
	temp_cam_rt = transform_point(tran_data, algorithm_param_global.camera_rt_eye);
	algorithm_param_global.camera_rt = temp_cam_rt;
	get_rt_inv(algorithm_param_global.camera_rt, algorithm_param_global.rt_inv);
	return true;
}

void visionObjectDetect::objecgt_pose_detect(camera_callback_data& cam_data, std::vector<std::string>& text_prompt) {
	//algorithm_param_global.cam_data = cam_data;
	LOG_WARN("hand_and_eye: {}", algorithm_param_global.hand_and_eye);
	algorithm_param_global.material_box.material_box_points.clear();
	cv::Mat sam_img;
	cv::resize(cam_data.color, sam_img, cam_data.dep.size());
	img_process_.set_background_image(sam_img);
	algorithm_param_global.result_img = sam_img.clone();
	std::thread send_thread(&samClient::send_request, &sam3_client_, std::ref(sam_img), text_prompt);
	send_thread.detach();
	/////////////////////////////////////////////////
	if(1 == algorithm_param_global.hand_and_eye){
		get_transform_matrix_eye_on_hand(algorithm_param_global);
	}
	algorithm_param_global.multiple_object_pose.clear();
	algorithm_param_global.result_error_data = 0;
	cv::Mat& result_color = algorithm_param_global.result_img;
	cv::Mat xyz_img;
	std::vector<yoloseg_detect_result> material_box_seg;
	std::vector<yoloseg_detect_result> sam_result;
	if (cam_data.color.empty() || cam_data.dep.empty() || cam_data.intrinsic.size() != 9 || cam_data.error_data != 0) {
		algorithm_param_global.result_error_data = -99;
		LOG_ERROR("cam_data_error!!");
		return;
	}
	cv::Mat yolo_img;
	cv::resize(cam_data.color, yolo_img, cam_data.dep.size());
	if (!img_process_.get_xyz_from_intrinsic(cam_data.dep, cam_data.intrinsic, cam_data.scale, xyz_img)) {
		algorithm_param_global.result_error_data = -115;
		LOG_ERROR("point_could_above_the_material_box!!!!!!");
		std::cout << "There is an object on the material " << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	//write_xyz_point(xyz_img);
	yoloseg_detect_result material_box;
	if (algorithm_param_global.material_box.is_detect) {
		std::vector<yoloseg_detect_result>  temp_result;
		yolo_detect_.onnx_detect(yolo_img, temp_result, material_box_seg);
		img_process_.draw_seg_result(material_box_seg, result_color, cv::Scalar(0, 255, 0));
		if (material_box_seg.size() < 1) {
			algorithm_param_global.result_error_data = -100;
			LOG_ERROR("box_result.size() <1");
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			return;
		}
		img_process_.find_correct_material_box(material_box_seg, result_color);
		if (material_box_seg.size() != 1) {
			algorithm_param_global.result_error_data = -101;
			LOG_ERROR("box_result.size() !=1");
			return;
		}
		material_box = material_box_seg[0];

		std::vector<cv::Point3f> box_points;
		if (!point_detect_.get_material_box_4point(material_box, xyz_img, cam_data.intrinsic, box_points)) {
			algorithm_param_global.result_error_data = -110;
			LOG_ERROR("get_material_box_4point_fail !");
			std::vector<sam3_result> sam_data;
			int error_data;
			sam3_client_.get_sam3_result_data(sam_data, error_data);
			return;
		}
		algorithm_param_global.material_box.material_box_points = box_points;
		img_process_.draw_box_limit_points(result_color, cam_data.intrinsic);
		if (!point_detect_.is_object_above_material_box(material_box, xyz_img)) {
			algorithm_param_global.result_error_data = -116;
			LOG_ERROR("object_above_material_box !!!!");
			return;
		}
	}

	get_sam3_detect_result(sam_result, material_box);
	img_process_.draw_seg_result(sam_result, result_color, cv::Scalar(0, 255, 0));
	mask_process_.remove_object_from_crop_region(sam_result);
	img_process_.draw_rio_dotted_line(result_color, algorithm_param_global.seg_rio.pts, cv::Scalar(255, 255, 0));
	
	mask_process_.select_items_in_material_box(material_box, sam_result);
	img_process_.draw_seg_result(sam_result, result_color, cv::Scalar(255, 0, 0));
	if (sam_result.size() < 1) {
		algorithm_param_global.result_error_data = -106;
		LOG_ERROR("sam_result.size() <1");
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}
	std::vector<int> object_index;
	if (!mask_process_.choose_correct_object_seg_hight(sam_result, xyz_img, object_index)) {
		LOG_ERROR("choose_correct_object_seg_fail!!!");
		algorithm_param_global.result_error_data = -102;
		return;
	}
	if (object_index.size() < 1) {
		LOG_ERROR("choose_box_from_pixel_z_fail_object_index_size<1!!!");
		algorithm_param_global.result_error_data = -102;
		return;
	}

	yoloseg_detect_result correct_object;
	for (int i = 0; i < object_index.size(); ++i) {
		correct_object = sam_result[object_index[i]];
		//img_process_.draw_objec_rect(result_color, correct_object);
		cv::Point2f object_mid_pix(0, 0);
		pcl::PointCloud<pcl::PointXYZ> object_point;
		mask_process_.get_object_downsampling_point(xyz_img, correct_object, object_point);
		if (object_point.size() < 100) {
			algorithm_param_global.result_error_data = -103;
			LOG_WARN("object_pix_count <100!!!");
			continue;
		}
		pcl::PointCloud<pcl::PointXYZ> object_points_stat;
		point_detect_.point_cloud_stat_removal(object_point.makeShared(), object_points_stat, 40, 0.8);
		if (object_points_stat.size() < 100) {
			algorithm_param_global.result_error_data = -104;
			LOG_WARN("object_points_stat.size() <100!!!");
			continue;
		}
		///////////////////////
		std::vector < cv::Point3f> angle_plane;
		std::vector<cv::Point3d> center_pt;
		std::vector<int> points_size;
		std::vector<cv::Point3i> plane_size;
		std::vector<cv::Point3f> polygon_vertex;
		if (!point_detect_.calculate_object_pose(correct_object, object_points_stat.makeShared(), center_pt, angle_plane, points_size, plane_size, polygon_vertex)) {
			algorithm_param_global.result_error_data = -112;
			LOG_WARN("calculate_object_pose_fail!!!");
			continue;
		}
		if (angle_plane.size() == center_pt.size() && angle_plane.size() == points_size.size()) {
			for (int j = 0; j < angle_plane.size(); ++j) {
				object_pose obj_pose(center_pt[j].x, center_pt[j].y, center_pt[j].z, angle_plane[j].x, angle_plane[j].y, angle_plane[j].z, points_size[j]);
				obj_pose.index = object_index[i];
				obj_pose.length = plane_size[j].x;
				obj_pose.width = plane_size[j].y;
				obj_pose.height = plane_size[j].z;
				obj_pose.polygon_vertex = polygon_vertex;
				if (!reuslt_post_processor_.is_outside_material_box(obj_pose,algorithm_param_global)) {
					algorithm_param_global.multiple_object_pose.push_back(obj_pose);
				}
				break;
			}
		}
		else {
			LOG_WARN("seg_plane_from_norm_fail_angle_plane.size()={}, center_pt.size()={}, points_size.size()={}",
				angle_plane.size(), center_pt.size(), points_size.size());
			algorithm_param_global.result_error_data = -113;
			continue;
		}
	}
	mask_process_.calculate_coverage_area(sam_result, algorithm_param_global.multiple_object_pose,xyz_img);
	img_process_.draw_multiple_pose_result(result_color, algorithm_param_global.multiple_object_pose, cam_data.intrinsic, cv::Scalar(0, 0, 255));
	if (algorithm_param_global.multiple_object_pose.size() > 0) {
		algorithm_param_global.result_error_data = 0;
	}
}

void visionObjectDetect::write_result_data() {
	result_save_io_.thread_write_result_image_data();
}