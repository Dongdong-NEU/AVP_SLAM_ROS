#ifndef SLAM_FILE_LOGGER_H
#define SLAM_FILE_LOGGER_H

#include <cstdio>
#include <fstream>
#include <mutex>
#include <string>
#include <ros/ros.h>

namespace AVP
{
namespace mapping
{

/**
 * 线程安全的 SLAM 文件日志器（单例）
 * 同时写入文件并输出到 ROS 控制台，便于离线分析建图/回环/优化过程
 */
class SlamFileLogger
{
public:
    static SlamFileLogger& Instance();

    /** 从私有 ROS 参数初始化日志路径，需在节点启动时调用一次 */
    void Init(ros::NodeHandle& private_nh);

    void Log(const std::string& category, const std::string& message);

    template<typename... Args>
    void Logf(const std::string& category, const char* fmt, Args... args)
    {
        char buffer[2048];
        snprintf(buffer, sizeof(buffer), fmt, args...);
        Log(category, buffer);
    }

    bool IsEnabled() const { return enabled_; }
    const std::string& LogFilePath() const { return log_file_path_; }

    /** 确保文件路径的父目录存在 */
    static bool EnsureParentDirectoryForFile(const std::string& file_path);

private:
    SlamFileLogger() = default;
    SlamFileLogger(const SlamFileLogger&) = delete;
    SlamFileLogger& operator=(const SlamFileLogger&) = delete;

    static bool EnsureDirectoryExists(const std::string& path);

    std::mutex mutex_;
    std::ofstream log_stream_;
    std::string log_file_path_;
    bool enabled_ = false;
    bool initialized_ = false;
};

} // namespace mapping
} // namespace AVP

#endif
