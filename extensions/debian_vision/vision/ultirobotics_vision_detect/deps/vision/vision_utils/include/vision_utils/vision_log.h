#pragma once
#ifndef VISION_LOGGER_H
#define VISION_LOGGER_H
#pragma once
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include <string>
#include <memory>
#include <thread>
#include <atomic>

class Logger {
public:
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    explicit Logger(const std::string& log_file_path)
        : log_file_path_(log_file_path), running_(true) {
        initLogger();
        startFlushThread();
    }

    ~Logger() {
        stopFlushThread();
        if (logger_) logger_->flush();
    }

    std::shared_ptr<spdlog::logger> getLogger() const {
        return logger_;
    }

private:
    std::string log_file_path_;
    std::shared_ptr<spdlog::logger> logger_;
    std::thread flush_thread_;
    std::atomic<bool> running_{ true };

    void initLogger() {
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            log_file_path_, 5 * 1024 * 1024, 3);

        static std::atomic<int> logger_count{ 0 };
        std::string logger_name = "vision_logger_" + std::to_string(logger_count++);

        logger_ = std::make_shared<spdlog::logger>(logger_name, file_sink);
        logger_->flush_on(spdlog::level::info);
        logger_->set_level(spdlog::level::debug);
        logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%t] [%l] %v");
    }

    void startFlushThread() {
        flush_thread_ = std::thread([this]() {
            while (running_) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                if (logger_) logger_->flush();
            }
            });
    }

    void stopFlushThread() {
        running_ = false;
        if (flush_thread_.joinable()) flush_thread_.join();
    }
};

// 通用宏（所有类通用）
#define LOG_INFO(...)  logger_->getLogger()->info(__VA_ARGS__)
#define LOG_DEBUG(...) logger_->getLogger()->debug(__VA_ARGS__)
#define LOG_WARN(...)  logger_->getLogger()->warn(__VA_ARGS__)
#define LOG_ERROR(...) logger_->getLogger()->error(__VA_ARGS__)


#endif // VISION_LOGGER_H
