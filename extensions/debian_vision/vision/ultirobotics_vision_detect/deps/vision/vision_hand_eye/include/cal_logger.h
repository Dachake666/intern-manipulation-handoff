#pragma once
#ifndef CAL_LOG_H
#define CAL_LOG_H
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h" // ��ѡ����ת��־�����ⵥ���ļ�����
#include<iostream>
extern std::string log_name;
//extern bool ensureDirectoryExistsRecursive(std::string& file_path);
class Logger {
public:
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    //static Logger& getInstance() {
    //    return instance;
   // }

    std::shared_ptr<spdlog::logger> getLogger() {
        return logger_;
    }

    // ֹͣ��ʱˢ���̣߳������˳�ǰ���ã�
    void stopFlushThread() {
        running_ = false;
        if (flush_thread_.joinable()) {
            flush_thread_.join();
        }
    }
    static Logger& instance_logger() {
        static Logger inst;
        return inst;
    }

private:
    //static Logger instance;
    std::shared_ptr<spdlog::logger> logger_;
    std::thread flush_thread_;       // ��ʱˢ���߳�
    std::atomic<bool> running_{ true }; // �߳����б�־

    Logger() {
        initLogger();
        startFlushThread(); // ������ʱˢ���߳�
    }

    ~Logger() {
        stopFlushThread(); // ����ʱֹͣ�߳�
        spdlog::shutdown(); // �ر���־����ȷ�������־ˢ��
    }

    void initLogger() {
        // ��ת�ļ���־�����̰߳�ȫ��
        //std::cout<<"-------------  "<<detect_server_name_global<<std::endl;
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            log_name,
            5 * 1024 * 1024, // 5MB ��ת
            3
        );

        logger_ = std::make_shared<spdlog::logger>("file_logger", file_sink);

        // ���ģ����ü��𴥷�ˢ�£�info�����ϼ�����־���������ˢ�£�
        logger_->flush_on(spdlog::level::info);

        // ��־����
        logger_->set_level(spdlog::level::debug);

        // ��־��ʽ��ʱ��+�߳�ID+����+���ݣ�
        logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%t] [%l] %v");

        // ����ΪĬ����־��������ȫ�ֵ��ã�
        spdlog::set_default_logger(logger_);
    }

    void startFlushThread() {
        flush_thread_ = std::thread([this]() {
            while (running_) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                logger_->flush();
            }
            });
    }
};

#define LOG_TRACE(...) spdlog::trace(__VA_ARGS__)
#define LOG_DEBUG(...) spdlog::debug(__VA_ARGS__)
#define LOG_INFO(...)  spdlog::info(__VA_ARGS__)
#define LOG_WARN(...)  spdlog::warn(__VA_ARGS__)
#define LOG_ERROR(...) spdlog::error(__VA_ARGS__)
#define LOG_CRITICAL(...) spdlog::critical(__VA_ARGS__)


//#define LOG_INFO(...) 
//#define LOG_WARN(...) 

#endif // !CAL_LOG_H