#include "sam3/sam3_client.h"
//#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <json/json.h>

// ZMQ配置
//std::string ZMQ_SERVER_ADDR = "tcp://172.24.1.134:5555";
samClient::samClient(){
    error_data_ = 0;
    is_get_data_success_ = 0;
    is_getting_ = 0;
    error_data_ = 0;
    got_data_ = 0;
    context_ = nullptr;
    socket_ = nullptr;
    is_connect_ = 0;
    //creat_samClient();
}

void samClient::creat_samClient(std::string server_ip) {
    is_connect_ = 0;
    std::string ZMQ_SERVER_ADDR = "tcp://" + server_ip;
    server_ip_ = server_ip;
    while (true) {
        try {
            if (context_ == nullptr)
                context_ = new zmq::context_t(1);
            socket_ = new zmq::socket_t(*context_, zmq::socket_type::req);

            socket_->connect(ZMQ_SERVER_ADDR);
            zmq_server_addr_ = ZMQ_SERVER_ADDR;
            socket_->set(zmq::sockopt::rcvtimeo, 10000);

            std::string test_data = "test";
            zmq::message_t send_msg(test_data.data(), test_data.size());
            socket_->send(send_msg, zmq::send_flags::none);

            zmq::message_t recv_msg;
            auto res = socket_->recv(recv_msg, zmq::recv_flags::none);
            if (res.has_value() && res.value() > 0) {
                is_connect_ = 1;
                std::cout << "ZMQ客户端连接成功:" << ZMQ_SERVER_ADDR << std::endl;
                break;
            }
            else {
                std::cout << "连接失败:" << ZMQ_SERVER_ADDR << std::endl;
            }

        }
        catch (const std::exception& e) {
            std::cerr << "ZMQ连接失败: " << e.what() << std::endl;
            delete socket_;
            socket_ = nullptr;
        }
    }
}

void samClient::thread_creat_sam_client(std::string server_ip) {
    std::thread(&samClient::creat_samClient, this, server_ip).detach();
}

samClient::~samClient() {
    if (socket_) {
        socket_->close();
        delete socket_;
    }
    if (context_) {
        context_->close();
        delete context_;
    }
}

void samClient::send_request(cv::Mat& img, std::vector<std::string> text_prompt) {
    {
        if (!is_connect_) {
            result_callback(std::vector<sam3_result>(), -204);
            return;
        }
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex_);
        if (is_getting_) {
            result_callback(std::vector<sam3_result>(), -205);
            return;
        }

        if (!socket_) {
            std::cerr << "ZMQ套接字未初始化" << std::endl;
            result_callback(std::vector<sam3_result>(), -200);
            return;
        }
        is_getting_ = true;
        is_get_data_success_ = false;
        got_data_ = false;
        error_data_ = 0;
    }

    try {
        cv::Mat safe_img = img.clone();
        size_t img_size = safe_img.total() * safe_img.elemSize();

        // 构造请求消息
        Json::Value header;
        header["type"] = "inference_request";
        header["image_shape"] = Json::Value(Json::arrayValue);
        header["image_shape"].append(img.rows);
        header["image_shape"].append(img.cols);
        header["image_shape"].append(img.channels());

        // 关键修改：将单个字符串改为JSON数组
        header["text_prompt"] = Json::Value(Json::arrayValue);
        for (const auto& prompt : text_prompt) {
            header["text_prompt"].append(prompt);
        }

        // 序列化JSON头
        Json::StreamWriterBuilder writer;
        std::string header_str = Json::writeString(writer, header);

        // 构造完整请求：JSON头 + \0分隔符 + 图像数据
        zmq::message_t request(header_str.size() + 1 + img_size);

        // 复制JSON头
        memcpy(request.data(), header_str.c_str(), header_str.size());
        // 复制\0分隔符
        memcpy(reinterpret_cast<char*>(request.data()) + header_str.size(), "\0", 1);
        // 复制图像数据
        memcpy(reinterpret_cast<char*>(request.data()) + header_str.size() + 1, safe_img.data, img_size);

        // 发送请求
        try {
            socket_->send(request, zmq::send_flags::none);
        }
        catch (const std::exception& e) {
            thread_creat_sam_client(server_ip_);
            throw e;
        }

        // 接收回复
        zmq::message_t reply;
        auto res = socket_->recv(reply, zmq::recv_flags::none);
        // 解析回复
        std::string reply_str(static_cast<char*>(reply.data()), reply.size());
        size_t header_end = reply_str.find('\0');
        if (header_end == std::string::npos) {
            throw std::runtime_error("回复格式错误：找不到JSON头结束符");
        }

        // 解析JSON头
        std::string reply_header_str = reply_str.substr(0, header_end);
        Json::Value reply_header;
        Json::CharReaderBuilder reader;
        std::string errs;
        std::istringstream s(reply_header_str);
        if (!Json::parseFromStream(reader, s, &reply_header, &errs)) {
            throw std::runtime_error("JSON解析失败: " + errs);
        }

        // 处理回复数据
        bool success = reply_header["success"].asBool();
        int count = reply_header["count"].asInt();

        std::vector<sam3_result> sam_data;
        if (success && count > 0) {
            // 解析boxes
            Json::Value boxes_json = reply_header["boxes"];
            std::vector<float> boxes;
            for (int i = 0; i < boxes_json.size(); ++i) {
                boxes.push_back(boxes_json[i].asFloat());
            }

            // 解析scores
            Json::Value scores_json = reply_header["scores"];
            std::vector<float> scores;
            for (int i = 0; i < scores_json.size(); ++i) {
                scores.push_back(scores_json[i].asFloat());
            }

            Json::Value cls_json = reply_header["cls_ids"];
            std::vector<int> cls_ids;
            for (int i = 0; i < cls_json.size(); ++i) {
                cls_ids.push_back(cls_json[i].asInt());
            }

            // 解析masks_shape
            Json::Value masks_shape_json = reply_header["masks_shape"];
            std::vector<std::pair<int, int>> masks_shape;
            for (int i = 0; i < masks_shape_json.size(); ++i) {
                int h = masks_shape_json[i][0].asInt();
                int w = masks_shape_json[i][1].asInt();
                masks_shape.emplace_back(h, w);
            }

            // 解析masks数据
            const char* masks_data_ptr = reply_str.c_str() + header_end + 1;
            size_t masks_data_size = reply.size() - header_end - 1;

            // 验证数据完整性
            if (boxes.size() != count * 4 || scores.size() != count || masks_shape.size() != count || cls_ids.size() != count) {
                throw std::runtime_error("回复数据不完整：boxes/scores/masks_shape数量不匹配");
            }

            // 处理每个检测结果
            size_t masks_offset = 0;
            for (int i = 0; i < count; ++i) {
                // 构造边界框
                cv::Rect box(boxes[4 * i], boxes[4 * i + 1], boxes[4 * i + 2] - boxes[4 * i], boxes[4 * i + 3] - boxes[4 * i + 1]);

                // 获取mask尺寸
                int mask_h = masks_shape[i].first;
                int mask_w = masks_shape[i].second;
                size_t mask_size = mask_h * mask_w;

                // 检查mask尺寸是否合法
                if (mask_h <= 0 || mask_w <= 0) {
                    throw std::runtime_error("无效的mask尺寸：" + std::to_string(mask_h) + "x" + std::to_string(mask_w));
                }

                // 检查masks_data_size是否足够
                if (masks_offset + mask_size > masks_data_size) {
                    throw std::runtime_error("masks数据不足：需要" + std::to_string(masks_offset + mask_size) + "字节，实际只有" + std::to_string(masks_data_size) + "字节");
                }

                // 构造mask
                cv::Mat mask(mask_h, mask_w, CV_8UC1);
                memcpy(mask.data, masks_data_ptr + masks_offset, mask_size);
                masks_offset += mask_size;

                // 添加到结果列表
                sam_data.emplace_back(box, scores[i], mask, cls_ids[i]);
            }

            // 调用结果回调
            result_callback(sam_data, 0);
        }
        else {
            // 调用结果回调（失败）
            result_callback(sam_data, -201);
        }
    }
    catch (const std::exception& e) {
        std::cerr << "ZMQ通信失败: " << e.what() << std::endl;
        result_callback(std::vector<sam3_result>(), -202);
    }
}

void samClient::result_callback(const std::vector<sam3_result>& result, int error_code) {
    std::lock_guard<std::mutex> lock(m_mutex_);
    
    error_data_ = error_code;
    
    if (error_code == 0 && !result.empty()) {
        sam_detect_data_ = result;
        is_get_data_success_ = 1;
    } else {
        is_get_data_success_ = 0;
        if (error_data_ == 0) {
            error_data_ = -203;
        }
    }
    is_getting_ = 0;
    got_data_ = 1;
    m_cv_.notify_one();
}

bool samClient::get_sam3_result_data(std::vector<sam3_result>& sam_data, int& error_data) {
    {
        std::unique_lock<std::mutex> lock(m_mutex_);
        m_cv_.wait_for(lock, std::chrono::seconds(10),
            [this] { return got_data_; });
        if (is_get_data_success_) {
            sam_data.swap(sam_detect_data_);
        }
        else {
            std::cout << " get_sam3_data_error!!!  " << error_data_ << std::endl;
        }
        error_data = error_data_;
    }
    return 1;
}

void get_show_seg_mat(std::vector<sam3_result>& result, cv::Mat& img) {
    for (int i = 0; i < result.size(); ++i) {
        cv::Mat mark = result[i].part_mask;
        int color_b = rand() % 256;
        int color_g = rand() % 256;
        int color_r = rand() % 256;

        uchar* img_data = img.data;
        uchar* m_data = mark.data;
        int x_start = result[i].box.x;
        int y_start = result[i].box.y;
        for (int y = 0; y < mark.rows; ++y)
            for (int x = 0; x < mark.cols; ++x) {
                if (m_data[y * mark.cols + x] > 0) {
                    img_data[(y + y_start) * img.cols * 3 + 3 * (x + x_start)] = 0.8 * img_data[(y + y_start) * img.cols * 3 + 3 * (x + x_start)] + 0.2 * color_b;
                    img_data[(y + y_start) * img.cols * 3 + 3 * (x + x_start) + 1] = 0.8 * img_data[(y + y_start) * img.cols * 3 + 3 * (x + x_start) + 1] + 0.2 * color_g;
                    img_data[(y + y_start) * img.cols * 3 + 3 * (x + x_start) + 2] = 0.8 * img_data[(y + y_start) * img.cols * 3 + 3 * (x + x_start) + 2] + 0.2 * color_r;
                }
            }

        cv::rectangle(img, result[i].box, cv::Scalar(0, 255, 0), 2);
    }
}

void samClient::test_sam_data() {
    //usleep(1000*1000);
    ////////////////////////////////////////////////
    //{
    //    cv::Mat img = cv::imread("bottle.png"); //
    //    std::string text_prompt = "bottle";    //
    //    send_request(img, text_prompt);
    //    std::vector<sam3_result> sam_data;
    //    int error_data;
    //    get_sam3_result_data(sam_data, error_data);
    //    std::cout << error_data << "   error_data_count " << sam_data.size() << std::endl;
    //    get_show_seg_mat(sam_data, img);
    //    cv::imwrite("out.png", img);
    //}
    //{
    //    cv::Mat img = cv::imread("color2025_11_11_19_29_40.png"); //
    //    std::string text_prompt = "clothes";    //
    //    send_request(img, text_prompt);
    //    std::vector<sam3_result> sam_data;
    //    int error_data;
    //    get_sam3_result_data(sam_data, error_data);
    //    std::cout << error_data << "   error_data_count " << sam_data.size() << std::endl;
    //    get_show_seg_mat(sam_data, img);
    //    cv::imwrite("out2.png", img);
    //}
    ////////////////////////////////////////////////
}

