#include"yolo/yolov8_seg.h"
yolov8seg::yolov8seg() {
    scale_ = 1.0f;
    top_ = 0;
    left_ = 0;
    nms_threshold_ = 0.45;
    num_classes_ = class_name_.size();
	

}

void yolov8seg::init(std::string model_path){
	//std::string model_path = model_path;
	session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_DISABLE_ALL);
	session_ = new Ort::Session(env_, model_path.c_str(), session_options_);
}

yolov8seg::~yolov8seg() {
    if (session_) {
        delete session_;
        session_ = nullptr;
    }
}

cv::Mat letterbox(const cv::Mat& src, int target_w, int target_h, float& scale, int& top, int& left) {
    int src_w = src.cols, src_h = src.rows;
    scale = std::min(target_w / (float)src_w, target_h / (float)src_h);
    int new_w = int(scale * src_w), new_h = int(scale * src_h);
    left = (target_w - new_w) / 2;
    top = (target_h - new_h) / 2;

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h));
    cv::Mat output(target_h, target_w, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(output(cv::Rect(left, top, new_w, new_h)));
    return output;
}

float calculate_iou(const cv::Rect& rect1, const cv::Rect& rect2) {
    cv::Rect intersection = rect1 & rect2;
    int intersection_area = intersection.width * intersection.height;
    int union_area = rect1.width * rect1.height + rect2.width * rect2.height - intersection_area;
    return static_cast<float>(intersection_area) / union_area;
}

std::vector<Detection> nms(std::vector<Detection>& detections, float nms_threshold) {
    std::vector<Detection> results;
    std::sort(detections.begin(), detections.end(), [](const Detection& a, const Detection& b) {
        return a.confidence > b.confidence;
        });

    for (const auto& det : detections) {
        bool keep = true;
        for (auto& res : results) {
            if (det.class_id == res.class_id) {
                float iou = calculate_iou(det.box, res.box);
                if (iou > nms_threshold) {
                    keep = false;
                    break;
                }
            }
        }
        if (keep) {
            results.push_back(det);
        }
    }
    return results;
}

void yolov8seg::get_input_tensor_values_data(cv::Mat& img, std::vector<float>& input_tensor_values) {
    scale_ = 1.0f;
    top_ = 0;
    left_ = 0;

    cv::Mat input_img = letterbox(img, INPUT_WIDTH, INPUT_HEIGHT, scale_, top_, left_);
    cv::cvtColor(input_img, input_img, cv::COLOR_BGR2RGB);

    uchar* input_data = (uchar*)input_img.data;
    int idx = 0;
    for (int c = 0; c < 3; ++c)
        for (int y = 0; y < INPUT_HEIGHT; ++y)
            for (int x = 0; x < INPUT_WIDTH; ++x)
                input_tensor_values[idx++] = 1.0 * input_data[3 * y * INPUT_WIDTH + 3 * x + c] /  255.0;//65535.0;//255.0;//255.0;
}

std::vector<Detection> yolov8seg::postprocess(const float* output, int ori_w, int ori_h) {
    std::vector<Detection> results;
    const int num_preds = 8400;
    const int stride = 4 + num_classes_ + 32;

    for (int i = 0; i < num_preds; ++i) {
        const float* ptr = output + i * stride;
        float cx = output[0 * num_preds + i];
        float cy = output[1 * num_preds + i];
        float w = output[2 * num_preds + i];
        float h = output[3 * num_preds + i];

        // ??????????????
        float max_conf = -1.0f;
        int class_id = -1;
        for (int j = 0; j < num_classes_; ++j) {
            float conf = output[(4 + j) * num_preds + i];
            if (conf > max_conf) {
                max_conf = conf;
                class_id = j;
            }
        }

        if (max_conf >= CONF_THRESH) {
            cx -= left_;
            cy -= top_;
            float x1 = (cx - w / 2) / scale_;
            float y1 = (cy - h / 2) / scale_;
            float x2 = (cx + w / 2) / scale_;
            float y2 = (cy + h / 2) / scale_;

            x1 = std::max(0.f, x1); y1 = std::max(0.f, y1);
            x2 = std::min((float)ori_w, x2); y2 = std::min((float)ori_h, y2);

            if (x2 > x1 && y2 > y1) {
                std::vector<float> mask_coeffs(32);
                for (int j = 0; j < 32; ++j) {
                    mask_coeffs[j] = output[(4 + num_classes_ + j) * num_preds + i];
                }
                results.push_back({ cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), class_id, max_conf, mask_coeffs });
            }
        }
    }
    return results;
}


cv::Mat yolov8seg::generate_mask(const std::vector<float>& mask_coeffs, cv::Mat& proto_mat, int proto_w, int proto_h,
    const cv::Rect& box, int ori_w, int ori_h) {
    const int mask_dim = mask_coeffs.size();  // ?? 32

    // 2. ?????????? [32, 1]
    cv::Mat coeffs_mat(mask_dim, 1, CV_32F);
    float* coeffs_data = (float*)coeffs_mat.data;
    for (int i = 0; i < mask_dim; ++i) {
        coeffs_data[i] = mask_coeffs[i];
    }

    // 3. ????????
    cv::Mat mask_flat = proto_mat * coeffs_mat;  // [H*W, 1]
    cv::Mat mask_flat_sigmoid;
    cv::exp(-mask_flat, mask_flat_sigmoid);  // Sigmoid ????
    mask_flat = 1.0 / (1.0 + mask_flat_sigmoid);
    //std::cout << "mask_flat size: " << mask_flat.size() << std::endl;
    //std::cout << "proto_w: " << proto_w << std::endl;
    //std::cout << "proto_h: " << proto_h << std::endl;
    cv::Mat mask = mask_flat.reshape(1, proto_h);  // [H, W]
    //std::cout << "mask size: " << mask.size() << " mask channel: " << mask.channels() << std::endl;
    // 4. ??????????????�� (640x640)
    cv::resize(mask, mask, cv::Size(INPUT_WIDTH, INPUT_HEIGHT), 0, 0, cv::INTER_LINEAR);
    //std::cout << "mask size: " << mask.size() << std::endl;
    // 5. ???????????????????????? padding ???
    int x1 = std::round((box.x * scale_) + left_);
    int y1 = std::round((box.y * scale_) + top_);
    int x2 = std::round(((box.x + box.width) * scale_) + left_);
    int y2 = std::round(((box.y + box.height) * scale_) + top_);

    // 6. ??????��????
    cv::Rect box_in_input(x1, y1, x2 - x1, y2 - y1);
    cv::Rect input_area(0, 0, INPUT_WIDTH, INPUT_HEIGHT);
    cv::Rect valid_box = box_in_input & input_area;

    if (valid_box.area() <= 0) {
        std::cout << "Warning: valid_box area <= 0\n";
        return cv::Mat();
    }

    // 7. ??????? crop ???????????? resize ???? bbox ??��
    cv::Mat crop = mask(valid_box);
    cv::resize(crop, crop, cv::Size(box.width, box.height), 0, 0, cv::INTER_LINEAR);

    // 8. ??????????????
    //cv::Mat reult_mat = cv::Mat(crop.size(), CV_8UC1);
    //uchar* reult_data = reult_mat.data;
    //float* crop_data = (float*)crop.data;
    //for(int y=0;y<crop.rows;++y)
    //    for (int x = 0; x < crop.cols; ++x) {
    //        if (crop_data[y * crop.cols + x] < 0.5)
    //            reult_data[y * crop.cols + x] = 0;
    //        else
    //            reult_data[y * crop.cols + x] = 255;
    //    }
    cv::Mat reult_mat = crop > 0.5;
    return reult_mat;
}

void yolov8seg::onnx_detect(cv::Mat& img, std::vector<yoloseg_detect_result>& reult, std::vector<yoloseg_detect_result>& box_reult) {
    std::vector<float> input_tensor_values(INPUT_WIDTH * INPUT_HEIGHT * 3);
    get_input_tensor_values_data(img, input_tensor_values);

    auto input_name = session_->GetInputNameAllocated(0, Ort::AllocatorWithDefaultOptions());
    auto output_name0 = session_->GetOutputNameAllocated(0, Ort::AllocatorWithDefaultOptions());
    auto output_name1 = session_->GetOutputNameAllocated(1, Ort::AllocatorWithDefaultOptions());

    std::array<int64_t, 4> input_shape{ 1, 3, INPUT_HEIGHT, INPUT_WIDTH };
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info_, input_tensor_values.data(), input_tensor_values.size(), input_shape.data(), input_shape.size()
    );

    std::vector<const char*> input_names{ input_name.get() };
    std::vector<const char*> output_names{ output_name0.get(), output_name1.get() };
    auto output_tensors = session_->Run(Ort::RunOptions{ nullptr }, input_names.data(), &input_tensor, 1, output_names.data(), 2);

    float* output0 = output_tensors[0].GetTensorMutableData<float>();  // [1, 4+cls+32, 8400]
    float* output1 = output_tensors[1].GetTensorMutableData<float>();  // [1, 32, 160, 160]
    std::vector<Detection> detections = postprocess(output0, img.cols, img.rows);
   
    detections = nms(detections, nms_threshold_);
    if (detections.size() > 0) {

        int size_wh = 160 * 160;
        cv::Mat proto_mat(size_wh, 32, CV_32F);
        float* proto_data = (float*)proto_mat.data;
        for (int c = 0; c < 32; ++c) {
            for (int i = 0; i < size_wh; ++i) {
                proto_data[i * 32 + c] = output1[c * size_wh + i];
            }
        }

        for (size_t i = 0; i < detections.size(); ++i) {
            cv::Mat mask = generate_mask(detections[i].mask_coeffs, proto_mat, 160, 160,
                detections[i].box, img.cols, img.rows);

            if (!mask.empty()) {
                yoloseg_detect_result tp;
                tp.box = detections[i].box;
                tp.class_id = detections[i].class_id;
                tp.confidence = detections[i].confidence;
                tp.part_mask = mask.clone();
                if (0 == tp.class_id)
                    box_reult.push_back(tp);
                else// if (1 == tp.class_id)
                    reult.push_back(tp);
            }
        }
    }
}
