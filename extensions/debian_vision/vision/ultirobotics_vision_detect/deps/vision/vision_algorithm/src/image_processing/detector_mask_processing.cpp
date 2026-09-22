#include "image_processing/detector_mask_processing.h"
#include"geometry_utils.h"

detectorMaskProcessor::detectorMaskProcessor(algorithmParam& param):algorithm_param_global(param){
}
detectorMaskProcessor::~detectorMaskProcessor() {
}

class obeject_attribute {
public:
	obeject_attribute() {}
	obeject_attribute(int t_i, int t_size, float t_z, float t_con) {
		i = t_i;
		size = t_size;
		z = t_z;
		confidence = t_con;
	}
	~obeject_attribute() {}
	obeject_attribute& operator=(const obeject_attribute& tp) {
		if (this != &tp) {
			z = tp.z;
			size = tp.size;
			i = tp.i;
			confidence = tp.confidence;
		}
		return *this;
	}
	float z;
	int size;
	int i;
	float confidence;
};

bool detectorMaskProcessor::choose_correct_object_seg(std::vector<yoloseg_detect_result>& detect_box, cv::Mat& xyz, std::vector<int>& out_index) {
	if (detect_box.size() < 1)
		return false;
	if (detect_box.size() == 1) {
		out_index.push_back(0);
		return true;
	}
	
	int object_size = detect_box.size();
	std::vector<float> vec_confidence;
	for (int i = 0; i < detect_box.size(); ++i) {
		vec_confidence.emplace_back(detect_box[i].confidence);
	}
	//std::sort(vec_confidence.begin(), vec_confidence.end());
	std::sort(vec_confidence.begin(), vec_confidence.end(), [](float a, float b) {
		return a > b;
		});
	int index_con = 0.5 * vec_confidence.size();

	if (index_con >= object_size)
		index_con = object_size - 1;

	if (index_con < 9) {
		if (object_size >= 10) {
			index_con = 9;
		}
		else {
			index_con = object_size - 1;
		}
	}

	float key_confidence = vec_confidence[index_con];
	//float key_confidence = vec_confidence[0.1* vec_confidence.size()];
	std::vector<obeject_attribute> obejct_att;

	uchar* img_data;
	int col;
	int row;
	//int index = 0;
	double average_z = 0;
	float* xyzdata = (float*)xyz.data;
	int x_start = 0;
	int y_start = 0;
	float temp_z = 0;
	std::vector<float> object_z;
	for (int i = 0; i < detect_box.size(); ++i) {
		object_z.clear();
		average_z = 0;
		img_data = detect_box[i].part_mask.data;
		col = detect_box[i].part_mask.cols;
		row = detect_box[i].part_mask.rows;
		x_start = detect_box[i].box.x;
		y_start = detect_box[i].box.y;
		for (int y = 0; y < row; ++y)
			for (int x = 0; x < col; ++x) {
				if (img_data[y * col + x] > 0) {
					temp_z = xyzdata[3 * (y + y_start) * xyz.cols + 3 * (x_start + x) + 2];
					if (temp_z < 10000) {
						object_z.push_back(temp_z);
					}
				}
			}
		if (object_z.size() < 200)
			continue;
		if (detect_box[i].confidence < key_confidence)
			continue;
		std::sort(object_z.begin(), object_z.end());
		average_z = object_z[object_z.size() * 0.8];
		obejct_att.push_back(obeject_attribute(i, object_z.size(), average_z, detect_box[i].confidence));
	}
	if (obejct_att.size() < 1)
		return false;

	int max_size = 0;
	for (int i = 0; i < obejct_att.size(); ++i) {
		if (max_size < obejct_att[i].size)
			max_size = obejct_att[i].size;
	}
	float value;
	float max_value = -10000000000;
	std::vector<std::pair<float, int>> vec_value;
	for (int i = 0; i < obejct_att.size(); ++i) {
		//value = obejct_att[i].z + 20.0 * obejct_att[i].size / max_size; //+ 50.0 * obejct_att[i].confidence;
		value = obejct_att[i].size;
		if (max_value < value) {
			max_value = value;
			//index = obejct_att[i].i;
		}
		vec_value.push_back(std::pair<float, int>(value, obejct_att[i].i));
	}
	std::sort(vec_value.begin(), vec_value.end(),[](const std::pair<float, int>& a, const std::pair<float, int>& b) {
			return a.first > b.first;
		});

	for (int i = 0; i < vec_value.size(); ++i) {
		if (i > 9)
			break;
		out_index.push_back(vec_value[i].second);
	}
	return true;
}

bool detectorMaskProcessor::choose_correct_object_seg_hight(std::vector<yoloseg_detect_result>& detect_box, cv::Mat& xyz, std::vector<int>& out_index){
	if (detect_box.size() < 1)
		return false;
	if (detect_box.size() == 1) {
		out_index.push_back(0);
		return true;
	}
    int number_key = algorithm_param_global.result_candidate_number - 1;
	if(number_key < 0)
		number_key = 0;
	int object_size = detect_box.size();
	std::vector<float> vec_confidence;
	for (int i = 0; i < detect_box.size(); ++i) {
		vec_confidence.emplace_back(detect_box[i].confidence);
	}
	//std::sort(vec_confidence.begin(), vec_confidence.end());
	std::sort(vec_confidence.begin(), vec_confidence.end(), [](float a, float b) {
		return a > b;
		});
	int index_con = 0.5 * vec_confidence.size();

	if (index_con >= object_size)
		index_con = object_size - 1;

	if (index_con < number_key) {
		if (object_size > number_key) {
			index_con = number_key;
		}
		else {
			index_con = object_size - 1;
		}
	}

	float key_confidence = vec_confidence[index_con];
	//float key_confidence = vec_confidence[0.1* vec_confidence.size()];
	std::vector<obeject_attribute> obejct_att;

	uchar* img_data;
	int col;
	int row;
	int index = 0;
	double average_z = 0;
	float* xyzdata = (float*)xyz.data;
	int x_start = 0;
	int y_start = 0;
	float temp_z = 0;
	std::vector<float> object_z;
	for (int i = 0; i < detect_box.size(); ++i) {
		object_z.clear();
		average_z = 0;
		img_data = detect_box[i].part_mask.data;
		col = detect_box[i].part_mask.cols;
		row = detect_box[i].part_mask.rows;
		x_start = detect_box[i].box.x;
		y_start = detect_box[i].box.y;
		for (int y = 0; y < row; ++y)
			for (int x = 0; x < col; ++x) {
				if (img_data[y * col + x] > 0) {
					if (y + y_start < 0 || y + y_start >= xyz.rows || x_start + x < 0 || x_start + x >= xyz.cols)
						continue;
					temp_z = xyzdata[3 * (y + y_start) * xyz.cols + 3 * (x_start + x) + 2];
					if (temp_z < 10000) {
						object_z.push_back(temp_z);
					}
				}
			}
		if (object_z.size() < 200)
			continue;
		if (detect_box[i].confidence < key_confidence)
			continue;
		std::sort(object_z.begin(), object_z.end());
		average_z = object_z[object_z.size() * 0.5];
		obejct_att.push_back(obeject_attribute(i, object_z.size(), average_z, detect_box[i].confidence));
	}
	if (obejct_att.size() < 1)
		return false;

	float value;
	float max_value = -10000000000;
	std::vector<std::pair<float, int>> vec_value;
	for (int i = 0; i < obejct_att.size(); ++i) {
		value = obejct_att[i].z;
		if (max_value < value) {
			max_value = value;
			index = obejct_att[i].i;
		}
		vec_value.push_back(std::pair<float, int>(value, obejct_att[i].i));
	}
	std::sort(vec_value.begin(), vec_value.end(), [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
		return a.first > b.first;
		});

	for (int i = 0; i < vec_value.size(); ++i) {
		if (i > number_key)
			break;
		out_index.push_back(vec_value[i].second);
	}
	return true;
}

void detectorMaskProcessor::select_items_in_material_box(yoloseg_detect_result& material_seg, std::vector<yoloseg_detect_result>& object_seg) {
	
	std::vector<yoloseg_detect_result> tmp_obj_result;
	cv::Rect overlap;
	int over_area;
	int obj_area;
	float ratio;
	float ratio_box;
	if (algorithm_param_global.material_box.is_detect) {
		tmp_obj_result.reserve(object_seg.size());

		float x;
		float y;
		float z;
		cv::Point2f result_pixel;
		std::vector<cv::Point2f> material_point2d;
		for (int i = 0; i < algorithm_param_global.material_box.material_box_points.size(); ++i) {
			x = algorithm_param_global.rt_inv[0] * algorithm_param_global.material_box.material_box_points[i].x + algorithm_param_global.rt_inv[1] * algorithm_param_global.material_box.material_box_points[i].y + algorithm_param_global.rt_inv[2] * algorithm_param_global.material_box.material_box_points[i].z + algorithm_param_global.rt_inv[3];
			y = algorithm_param_global.rt_inv[4] * algorithm_param_global.material_box.material_box_points[i].x + algorithm_param_global.rt_inv[5] * algorithm_param_global.material_box.material_box_points[i].y + algorithm_param_global.rt_inv[6] * algorithm_param_global.material_box.material_box_points[i].z + algorithm_param_global.rt_inv[7];
			z = algorithm_param_global.rt_inv[8] * algorithm_param_global.material_box.material_box_points[i].x + algorithm_param_global.rt_inv[9] * algorithm_param_global.material_box.material_box_points[i].y + algorithm_param_global.rt_inv[10] * algorithm_param_global.material_box.material_box_points[i].z + algorithm_param_global.rt_inv[11];
			result_pixel.x = algorithm_param_global.cam_data.intrinsic[0] * x / z + algorithm_param_global.cam_data.intrinsic[2];
			result_pixel.y = algorithm_param_global.cam_data.intrinsic[4] * y / z + algorithm_param_global.cam_data.intrinsic[5];
			material_point2d.push_back(result_pixel);
		}
		if (material_point2d.size() < 4)
			return;
		float threshold_dist = 0;

	float min_lenght = 100000000;
	float dist = 0;
	for (int i = 0; i < material_point2d.size(); ++i) {
		dist = std::sqrt((material_point2d[(i + 1) % material_point2d.size()].x - material_point2d[i].x) * (material_point2d[(i + 1) % material_point2d.size()].x - material_point2d[i].x) +
			(material_point2d[(i + 1) % material_point2d.size()].y - material_point2d[i].y) * (material_point2d[(i + 1) % material_point2d.size()].y - material_point2d[i].y));
		if (min_lenght > dist)
			min_lenght = dist;
	}
	threshold_dist = -min_lenght * 0.02;


	cv::Point2f temp_center;
	bool is_out = 0;
	for (int i = 0; i < object_seg.size(); ++i) {
		is_out = 0;
		overlap = object_seg[i].box & material_seg.box;
		over_area = overlap.area();
		obj_area = object_seg[i].box.area();
		ratio = 1.0 * over_area / obj_area;
		ratio_box = 1.0 * over_area / material_seg.box.area();
		///////////////////
		temp_center.x = object_seg[i].box.x + object_seg[i].box.width / 2;
		temp_center.y = object_seg[i].box.y + object_seg[i].box.height / 2;
		if (ratio_box > 0.5)
			continue;
		for (int j = 0; j < material_point2d.size(); ++j) {
			float dist = pointToLineDistance(temp_center, material_point2d[j], material_point2d[(j + 1) % material_point2d.size()]);
			if (dist > threshold_dist) {
				is_out = 1;
				break;
			}
		}

			///////////////////
			if (ratio > 0.9 && 0 == is_out) {
				tmp_obj_result.push_back(object_seg[i]);
			}
		}
		//if (tmp_obj_result.size() < 1)
		//	return;
	}
	else {
		tmp_obj_result.swap(object_seg);
	}
	if (tmp_obj_result.size() < 1) {
		tmp_obj_result.swap(object_seg);
		return;
	}

	std::sort(tmp_obj_result.begin(), tmp_obj_result.end(), [](const auto& a, const auto& b) {return a.confidence > b.confidence; });
	std::vector<yoloseg_detect_result> target_obj_result;
	bool found = 0;
	for (int i = 0; i < tmp_obj_result.size(); ++i) {
		found = 0;
		for (int j = 0; j < target_obj_result.size(); ++j) {
			if (tmp_obj_result[i].class_id == target_obj_result[j].class_id)
				continue;
			overlap = tmp_obj_result[i].box & target_obj_result[j].box;
			over_area = overlap.area();
			ratio = 1.0 * over_area / tmp_obj_result[i].box.area();
			ratio_box = 1.0 * over_area / target_obj_result[j].box.area();
			if (ratio > 0.8 && ratio_box > 0.8)
				found = 1;
		}
		if (!found) {
			target_obj_result.push_back(tmp_obj_result[i]);
		}
	}
	/////////////////////////////////////////////
	object_seg.swap(target_obj_result);

}

bool detectorMaskProcessor::get_object_downsampling_point(cv::Mat& xyz_img, yoloseg_detect_result& object, pcl::PointCloud<pcl::PointXYZ>& object_point) {
	uchar* mdata = object.part_mask.data;
	int col = object.part_mask.cols;
	int row = object.part_mask.rows;
	int x_start = object.box.x;
	int y_start = object.box.y;
	int count = 0;
	float* xyz_data = (float*)xyz_img.data;
	float dep_z = 0;
	int xyz_x = 0;
	int xyz_y = 0;
	for (int y = 0; y < row; ++y)
		for (int x = 0; x < col; ++x) {
			if (mdata[y * col + x] > 0) {
				xyz_x = x_start + x;
				xyz_y = y + y_start;
				if (xyz_x >= 0 && xyz_x < xyz_img.cols && xyz_y >= 0 && xyz_y < xyz_img.rows) {
					dep_z = xyz_data[3 * (xyz_y)*xyz_img.cols + 3 * xyz_x + 2];
					if (dep_z < 1000000)
						count++;
				}
			}
		}
	if (count < 10)
		return false;
	int ratio = sqrt(count / 1500);
	if (ratio < 1)
		ratio = 1;

	pcl::PointXYZ tmp_p;
	for (int y = 0; y < row; y = y + ratio)
		for (int x = 0; x < col; x = x + ratio) {
			if (mdata[y * col + x] > 0) {
				xyz_x = x_start + x;
				xyz_y = y + y_start;
				if (xyz_x >= 0 && xyz_x < xyz_img.cols && xyz_y >= 0 && xyz_y < xyz_img.rows) {
					tmp_p.x = xyz_data[3 * (xyz_y) * xyz_img.cols + 3 * (xyz_x)];
					tmp_p.y = xyz_data[3 * (xyz_y) * xyz_img.cols + 3 * (xyz_x) + 1];
					tmp_p.z = xyz_data[3 * (xyz_y) * xyz_img.cols + 3 * (xyz_x) + 2];
					if (tmp_p.z < 10000) {
						tmp_p.x = tmp_p.x;
						tmp_p.y = tmp_p.y;
						tmp_p.z = tmp_p.z;
						object_point.push_back(tmp_p);
					}
				}
			}
		}
	return true;
}

cv::Vec3d eulerToPlaneNormal(double rx_angle, double ry_angle, double rz_angle) {
	double rx = rx_angle * CV_PI / 180.0;
	double ry = ry_angle * CV_PI / 180.0;
	double rz = rz_angle * CV_PI / 180.0;

	cv::Mat R;
	cv::Rodrigues(cv::Vec3d(rx, ry, rz), R);
	cv::Mat n = R * cv::Mat(cv::Vec3d(0, 0, 1));
	return cv::Vec3d(n.at<double>(0), n.at<double>(1), n.at<double>(2));
}

cv::Point3d pixelToWorldPlanePoint(
	cv::Point2d pixel,
	algorithmParam& alg_param,
	cv::Vec3d normal,
	const cv::Point3d& plane_world) {
	cv::Vec3d dir_cam(
		(pixel.x - alg_param.cam_data.intrinsic[2]) / alg_param.cam_data.intrinsic[0],
		(pixel.y - alg_param.cam_data.intrinsic[5]) / alg_param.cam_data.intrinsic[4],
		1.0
	);
	cv::Point3d cam_origin_world;
	cam_origin_world.x = alg_param.camera_rt[0][3];
	cam_origin_world.y = alg_param.camera_rt[1][3];
	cam_origin_world.z = alg_param.camera_rt[2][3];
	// ===================== 3. 相机方向 → 世界坐标系方向 =====================
	cv::Point3f dir_world;
	dir_world.x = alg_param.camera_rt[0][0] * dir_cam[0] + alg_param.camera_rt[0][1] * dir_cam[1] + alg_param.camera_rt[0][2] * dir_cam[2];
	dir_world.y = alg_param.camera_rt[1][0] * dir_cam[0] + alg_param.camera_rt[1][1] * dir_cam[1] + alg_param.camera_rt[1][2] * dir_cam[2];
	dir_world.z = alg_param.camera_rt[2][0] * dir_cam[0] + alg_param.camera_rt[2][1] * dir_cam[1] + alg_param.camera_rt[2][2] * dir_cam[2];

	// 3. 平面方程
	//cv::Vec3d n = eulerToPlaneNormal(plane_rx, plane_ry, plane_rz);
	double nx = normal[0], ny = normal[1], nz = normal[2];
	double X0 = plane_world.x, Y0 = plane_world.y, Z0 = plane_world.z;
	double d = -(nx * X0 + ny * Y0 + nz * Z0);

	// 5. 求交
	double denominator = nx * dir_world.x + ny * dir_world.y + nz * dir_world.z;
	if (fabs(denominator) < 1e-6) {
		return cv::Point3d(NAN, NAN, NAN);
	}

	double s = -(nx * cam_origin_world.x + ny * cam_origin_world.y + nz * cam_origin_world.z + d) / denominator;

	// 结果
	cv::Point3d p_world;
	p_world.x = cam_origin_world.x + s * dir_world.x;
	p_world.y = cam_origin_world.y + s * dir_world.y;
	p_world.z = cam_origin_world.z + s * dir_world.z;

	return p_world;
}

void detectorMaskProcessor::calculate_coverage_area(std::vector<yoloseg_detect_result>& detect_result, std::vector<object_pose>& pose,cv::Mat& xyz){
	int index = 0;
	std::vector<int> object_hight;
	float* xyz_data = (float*)xyz.data;
	cv::Point2d center_pt(0, 0);
	cv::Point2i line_segment_pt;
	cv::Point2i pose_box_xy;
	cv::Point3d pose_tmp_pt;
	bool found_obejct_side_pt = 0;
	for (int ijk = 0; ijk < pose.size(); ++ijk) {
		cv::Vec3d normal = eulerToPlaneNormal(pose[ijk].angle_x, pose[ijk].angle_y, pose[ijk].angle_z);
		cv::Point3d plane_world(pose[ijk].x, pose[ijk].y, pose[ijk].z);
		index = pose[ijk].index;
		std::vector<cv::Point> img_pts;
		cv::Mat mask = detect_result[index].part_mask;
		pose_box_xy.x = detect_result[index].box.x;
		pose_box_xy.y = detect_result[index].box.y;
		cv::Mat show = mask.clone();
		uchar* show_data = show.data;
		uchar* data = mask.data;
		for (int i = 0; i < mask.rows; ++i)
			for (int j = 0; j < mask.cols; ++j) {
				if (data[i * mask.cols + j] > 0) {
					img_pts.emplace_back(j, i);
					show_data[i * mask.cols + j] = 255;
				}
			}
		cv::RotatedRect min_rect = cv::minAreaRect(img_pts);
		std::vector<cv::Point> hull;
		cv::convexHull(img_pts, hull);
		cv::Point2f rect_vertices[4];
		min_rect.points(rect_vertices);
		cv::Rect src_rect = detect_result[index].box;
		cv::Point2f rect_center_pt(0, 0);
		for (int i = 0; i < 4; ++i) {
			rect_vertices[i].x = rect_vertices[i].x + src_rect.x;
			rect_vertices[i].y = rect_vertices[i].y + src_rect.y;
			rect_center_pt.x += rect_vertices[i].x;
			rect_center_pt.y += rect_vertices[i].y;
		}
		rect_center_pt.x = rect_center_pt.x / 4;
		rect_center_pt.y = rect_center_pt.y / 4;

		cv::Point2f rect_vertices_temp[4];
		min_rect.points(rect_vertices_temp);

		cv::Rect overlap;
		int over_area;
		cv::Point2i p;
		uchar* d_data;
		bool inside = false;
		int inside_count = 0;
		int other_count = 0;
		float ratio_overlap = 0;
		int inside_z = 0;
		cv::Point2i min_dist_pt;
		float min_dist = 100000;
		for (int i = 0; i < detect_result.size(); ++i) {
			if (i == index)
				continue;
			object_hight.clear();
			center_pt = cv::Point2d(0, 0);
			overlap = detect_result[i].box & src_rect;
			over_area = overlap.area();
			if (over_area < 5)
				continue;
			inside_count = 0;
			other_count = 0;
			d_data = detect_result[i].part_mask.data;
			for (int y = 0; y < detect_result[i].part_mask.rows; ++y)
				for (int x = 0; x < detect_result[i].part_mask.cols; ++x) {
					if (d_data[y * detect_result[i].part_mask.cols + x] > 0) {
						other_count++;
						p.x = x + detect_result[i].box.x;
						p.y = y + detect_result[i].box.y;

						inside = false;
						for (int ik = 0, jk = 3; ik < 4; jk = ik++)
						{
							const cv::Point2f& vi = rect_vertices[ik];
							const cv::Point2f& vj = rect_vertices[jk];
							if (((vi.y > p.y) != (vj.y > p.y)) &&
								(p.x < (vj.x - vi.x) * (p.y - vi.y) / (vj.y - vi.y) + vi.x))
							{
								inside = !inside;
							}
						}
						if (inside) {
							if (p.y < 0 || p.y >= xyz.rows || p.x < 0 || p.x >= xyz.cols)
								continue;
							if (xyz_data[3 * p.y * xyz.cols + 3 * p.x + 2] < 10000) {
								inside_count++;
								object_hight.push_back(xyz_data[3 * p.y * xyz.cols + 3 * p.x + 2]);
								center_pt.x += p.x;
								center_pt.y += p.y;
							}
						}
					}
				}
			if (inside_count < 10)
				continue;
			center_pt.x = center_pt.x / inside_count;
			center_pt.y = center_pt.y / inside_count;
			std::sort(object_hight.begin(), object_hight.end());
			inside_z = object_hight[object_hight.size() / 2];
			found_obejct_side_pt = 0;
			for (float ratio_li = 0; ratio_li < 2; ratio_li = ratio_li + 0.05) {
				line_segment_pt.x = ratio_li * rect_center_pt.x + (1 - ratio_li) * center_pt.x;
				line_segment_pt.y = ratio_li * rect_center_pt.y + (1 - ratio_li) * center_pt.y;
				if (line_segment_pt.y - pose_box_xy.y < 0 || line_segment_pt.y - pose_box_xy.y >= mask.rows || line_segment_pt.x - pose_box_xy.x < 0 || line_segment_pt.x - pose_box_xy.x >= mask.cols)
					continue;
				if (data[(line_segment_pt.y - pose_box_xy.y) * mask.cols + line_segment_pt.x - pose_box_xy.x] > 0) {
					int y_start_j = line_segment_pt.y - pose_box_xy.y - 3;
					int y_end_j = line_segment_pt.y - pose_box_xy.y + 3;
					int x_start_i = line_segment_pt.x - pose_box_xy.x - 3;
					int x_end_i = line_segment_pt.x - pose_box_xy.x + 3;
					int temp_z;
					int count_tp = 0;
					pose_tmp_pt.x = 0;
					pose_tmp_pt.y = 0;
					pose_tmp_pt.z = 0;
					for (int y = y_start_j; y < y_end_j; ++y)
						for (int x = x_start_i; x < x_end_i; ++x) {
							if (y < 0 || y >= mask.rows || x < 0 || x >= mask.cols)
								continue;
							if (data[y * mask.cols + x] > 0) {
								temp_z = xyz_data[3 * (y + pose_box_xy.y) * xyz.cols + 3 * (x + pose_box_xy.x) + 2];
								if (fabs(temp_z - plane_world.z) < 100) {
									pose_tmp_pt.x += xyz_data[3 * (y + pose_box_xy.y) * xyz.cols + 3 * (x + pose_box_xy.x)];
									pose_tmp_pt.y += xyz_data[3 * (y + pose_box_xy.y) * xyz.cols + 3 * (x + pose_box_xy.x) + 1];
									pose_tmp_pt.z += temp_z;
									count_tp++;
								}
							}
						}
					if (count_tp > 10) {
						pose_tmp_pt.x = pose_tmp_pt.x / count_tp;
						pose_tmp_pt.y = pose_tmp_pt.y / count_tp;
						pose_tmp_pt.z = pose_tmp_pt.z / count_tp;
						found_obejct_side_pt = 1;
						break;
					}
				}
			}

			if (0 == found_obejct_side_pt)
				pose_tmp_pt = plane_world;
			cv::Point3d plane_pt = pixelToWorldPlanePoint(center_pt, algorithm_param_global, normal,
				pose_tmp_pt);

			ratio_overlap = 1.0 * inside_count / other_count;

			if (ratio_overlap > 0.02 && inside_z > plane_pt.z) {
				pose[ijk].overlap_area_ratio.push_back(ratio_overlap);
				pose[ijk].overlap_index.push_back(i);
			}

		}
	}

	int found = -1;
	for (int ijk = 0; ijk < pose.size(); ++ijk) {
		for (int i = 0; i < pose[ijk].overlap_index.size(); ++i) {
			found = -1;
			for (int j = 0; j < pose.size(); ++j) {
				if (ijk == j)
					continue;
				if (pose[ijk].overlap_index[i] == pose[j].index) {
					found = j;
				}
			}
			pose[ijk].overlap_index[i] = found;
		}
	}
}


void detectorMaskProcessor::remove_object_from_crop_region(std::vector<yoloseg_detect_result>& detect_box) {
	if (!algorithm_param_global.seg_rio.is_using)
		return;
	std::vector<cv::Point2f>  crop_box_img_pt = algorithm_param_global.seg_rio.pts;
	for (size_t i = 0; i < crop_box_img_pt.size(); ++i){
		crop_box_img_pt[i].x = crop_box_img_pt[i].x * algorithm_param_global.result_img.cols;
		crop_box_img_pt[i].y = crop_box_img_pt[i].y * algorithm_param_global.result_img.rows;
	}
	if (detect_box.size() < 1)
		return;

 	cv::Point2f temp_pts;
	std::vector<yoloseg_detect_result> out_seg_detect;
	int x_start, x_end, y_start, y_end;
	int x_step, y_step;
	int inside_count = 0;
	int count_all = 0;
	float ratio_inside = 0;
	for (int i = 0; i < detect_box.size(); ++i) {
		inside_count = 0;
		count_all = 0;
		x_start = detect_box[i].box.x;
		x_end = detect_box[i].box.x + detect_box[i].box.width;
		y_start = detect_box[i].box.y;
		y_end = detect_box[i].box.y + detect_box[i].box.height;
		x_step = detect_box[i].box.width / 10;
		y_step = detect_box[i].box.height / 10;
		if (x_step < 1)
			x_step = 1;
		if (y_step < 1)
			y_step = 1;
		for (int x = x_start; x < x_end; x = x + x_step) {
			for (int y = y_start; y < y_end; y = y + y_step) {
				temp_pts.x = x;
				temp_pts.y = y;
				if (is_point_in_polygon(temp_pts, crop_box_img_pt)) {
					inside_count++;
				}
				count_all++;
			}
		}
		ratio_inside = 1.0 * inside_count / count_all;
		if (ratio_inside >= algorithm_param_global.seg_rio.inside_ratio) {
			out_seg_detect.push_back(detect_box[i]);
		}
	}
	detect_box.swap(out_seg_detect);
}