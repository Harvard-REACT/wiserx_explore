#ifndef CUSTOM_LOGGER_H
#define CUSTOM_LOGGER_H

#include <string>
#include <fstream>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <cstdarg>
#include <memory>
#include <iostream>
#include <map>
#include <sstream>
#include <cstdlib>
#include <pwd.h>
#include <unistd.h>
#include <ros/ros.h>
#include <algorithm>

enum LogLevel {
    DEBUG,
    INFO,
    WARN,
    ERROR
};

class CustomLogger {
public:
    static CustomLogger& getInstance() {
        static CustomLogger instance;
        return instance;
    }

    template<typename... Args>
    void log(LogLevel level, const char* file, int line, const char* format, Args... args) {
        if (level < minLevel_) return;

        std::lock_guard<std::mutex> lock(mutex_);
        if (log_file_.is_open()) {
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            
            log_file_ << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X") << " ";

            std::string path(file);
            const size_t last_slash_idx = path.find_last_of("/");
            if (std::string::npos != last_slash_idx) {
                path.erase(0, last_slash_idx + 1);
            }
            log_file_ << levelToString(level) << " [" << path << ":" << line << "] ";
            
            size_t size = std::snprintf(nullptr, 0, format, args...) + 1;
            std::unique_ptr<char[]> buf(new char[size]);
            std::snprintf(buf.get(), size, format, args...);
            
            log_file_ << buf.get() << std::endl;
        }
    }

    template<typename... Args>
    void log_throttle(LogLevel level, const char* file, int line, double duration, const char* format, Args... args) {
        if (level < minLevel_) return;

        auto now = std::chrono::steady_clock::now();
        std::string key = std::string(file) + ":" + std::to_string(line);

        bool should_log = false;
        {
            std::lock_guard<std::mutex> lock(throttle_mutex_);
            if (last_log_times_.find(key) == last_log_times_.end() ||
                std::chrono::duration_cast<std::chrono::duration<double>>(now - last_log_times_[key]).count() > duration)
            {
                last_log_times_[key] = now;
                should_log = true;
            }
        }

        if (should_log) {
            log(level, file, line, format, args...);
        }
    }
    
    void setMinLevel(LogLevel level) {
        minLevel_ = level;
    }

private:
    CustomLogger() : minLevel_(INFO) {
        const char* home_dir = getenv("HOME");
        if (home_dir == nullptr) {
            struct passwd* pw = getpwuid(getuid());
            if (pw) {
                home_dir = pw->pw_dir;
            }
        }

        if (home_dir == nullptr) {
            std::cerr << "FATAL: Could not determine home directory. Cannot create log file." << std::endl;
            std::exit(EXIT_FAILURE);
        }

        std::string log_dir_path = std::string(home_dir) + "/catkin_ws/src/m-explore/explore/data/data_wiserx/";

        std::string node_identifier = "non_ros_node";
        if (ros::isInitialized()) {
            node_identifier = ros::this_node::getName();
            // Clean up the node name for use in a filename
            if (!node_identifier.empty() && node_identifier.front() == '/') {
                node_identifier.erase(0, 1);
            }
            std::replace(node_identifier.begin(), node_identifier.end(), '/', '_');
        }

        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << log_dir_path << "explore_log_" << node_identifier << "_" << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S") << ".log";
        log_file_.open(ss.str(), std::ios::out | std::ios::app);
        if (!log_file_.is_open()) {
            std::cerr << "FATAL: Failed to open log file: " << ss.str() << std::endl;
            std::cerr << "Please ensure the directory exists and has write permissions." << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }

    ~CustomLogger() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

    CustomLogger(const CustomLogger&) = delete;
    CustomLogger& operator=(const CustomLogger&) = delete;

    const char* levelToString(LogLevel level) {
        switch (level) {
            case DEBUG: return "[DEBUG]";
            case INFO:  return "[INFO] ";
            case WARN:  return "[WARN] ";
            case ERROR: return "[ERROR]";
            default:    return "[-----]";
        }
    }

    std::ofstream log_file_;
    std::mutex mutex_;
    LogLevel minLevel_;

    std::mutex throttle_mutex_;
    std::map<std::string, std::chrono::steady_clock::time_point> last_log_times_;
};

#define CUSTOM_LOG_DEBUG(...) CustomLogger::getInstance().log(DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#define CUSTOM_LOG_INFO(...)  CustomLogger::getInstance().log(INFO, __FILE__, __LINE__, __VA_ARGS__)
#define CUSTOM_LOG_WARN(...)  CustomLogger::getInstance().log(WARN, __FILE__, __LINE__, __VA_ARGS__)
#define CUSTOM_LOG_ERROR(...) CustomLogger::getInstance().log(ERROR, __FILE__, __LINE__, __VA_ARGS__)

#define CUSTOM_LOG_DEBUG_THROTTLE(duration, ...) CustomLogger::getInstance().log_throttle(DEBUG, __FILE__, __LINE__, duration, __VA_ARGS__)
#define CUSTOM_LOG_INFO_THROTTLE(duration, ...)  CustomLogger::getInstance().log_throttle(INFO, __FILE__, __LINE__, duration, __VA_ARGS__)
#define CUSTOM_LOG_WARN_THROTTLE(duration, ...)  CustomLogger::getInstance().log_throttle(WARN, __FILE__, __LINE__, duration, __VA_ARGS__)
#define CUSTOM_LOG_ERROR_THROTTLE(duration, ...) CustomLogger::getInstance().log_throttle(ERROR, __FILE__, __LINE__, duration, __VA_ARGS__)

#endif // CUSTOM_LOGGER_H