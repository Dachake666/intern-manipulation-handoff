#ifndef SAM3_CLIENT_H
#define SAM3_CLIENT_H
#include<iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include "zmq.hpp"

class sam3_result {
public:
    sam3_result() {
    }
    sam3_result(cv::Rect box_tp,float con_tp,cv::Mat& mask,int id) {
        box = box_tp;
        confidence = con_tp;
        part_mask = mask.clone();
        cls_id = id;
    }
    ~sam3_result() {
    }
    sam3_result& operator=(const sam3_result& tp) {
        if (this != &tp) {
            box = tp.box;
            confidence = tp.confidence;
            part_mask = tp.part_mask.clone();
            cls_id = tp.cls_id;
        }
        return *this;
    }
    cv::Rect box;
    float confidence;
    cv::Mat part_mask;
    int cls_id;
};

class samClient{
public:
    samClient();
    void creat_samClient(std::string server_ip);
    void thread_creat_sam_client(std::string server_ip);
    ~samClient();
    void send_request(cv::Mat& img, std::vector<std::string> text_prompt);
    bool get_sam3_result_data(std::vector<sam3_result>& sam_data,int& error_data);
    void test_sam_data();
private:
    void result_callback(const std::vector<sam3_result>& result, int error_code);
    
    std::vector<sam3_result> sam_detect_data_;
    int error_data_;
    
    std::mutex m_mutex_;
    bool got_data_;
    std::atomic<bool> is_getting_{ 0 };
    std::atomic<bool> is_get_data_success_{0};
    std::condition_variable m_cv_;
    
    // ZMQ相关成员
    zmq::context_t* context_;
    zmq::socket_t* socket_;
    std::string zmq_server_addr_;

    std::atomic<bool> is_connect_{ 0 };
    std::string server_ip_;
};

#endif
