#include "slam_file_logger.h"

#include <ros/package.h>
#include <sys/stat.h>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace AVP
{
namespace mapping
{

SlamFileLogger& SlamFileLogger::Instance()
{
    static SlamFileLogger instance;
    return instance;
}

bool SlamFileLogger::EnsureDirectoryExists(const std::string& path)
{
    if (path.empty())
    {
        return false;
    }

    struct stat st;
    if (stat(path.c_str(), &st) == 0)
    {
        return S_ISDIR(st.st_mode);
    }

    const size_t pos = path.find_last_of('/');
    if (pos != std::string::npos)
    {
        const std::string parent = path.substr(0, pos);
        if (!parent.empty() && !EnsureDirectoryExists(parent))
        {
            return false;
        }
    }

    if (mkdir(path.c_str(), 0755) == 0)
    {
        return true;
    }

    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool SlamFileLogger::EnsureParentDirectoryForFile(const std::string& file_path)
{
    const size_t pos = file_path.find_last_of('/');
    if (pos == std::string::npos)
    {
        return true;
    }
    return EnsureDirectoryExists(file_path.substr(0, pos));
}

void SlamFileLogger::Init(ros::NodeHandle& private_nh)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_)
    {
        return;
    }

    std::string log_dir;
    std::string log_file_name;
    private_nh.param<std::string>("log_dir", log_dir, "");
    private_nh.param<std::string>("log_file", log_file_name, "mapping_slam.log");
    private_nh.param<bool>("enable_file_log", enabled_, true);

    if (log_dir.empty())
    {
        log_dir = ros::package::getPath("avp") + "/output/logs";
    }

    if (!EnsureDirectoryExists(log_dir))
    {
        ROS_ERROR("Failed to create log directory: %s", log_dir.c_str());
        enabled_ = false;
        initialized_ = true;
        return;
    }

    log_file_path_ = log_dir + "/" + log_file_name;
    log_stream_.open(log_file_path_, std::ios::out | std::ios::app);
    if (!log_stream_.is_open())
    {
        ROS_ERROR("Failed to open log file: %s", log_file_path_.c_str());
        enabled_ = false;
        initialized_ = true;
        return;
    }

    initialized_ = true;
    const std::time_t now = std::time(nullptr);
    log_stream_ << "========== SLAM session started at "
                << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S")
                << " ==========" << std::endl;
    log_stream_.flush();

    ROS_INFO("SLAM file log enabled: %s", log_file_path_.c_str());
}

void SlamFileLogger::Log(const std::string& category, const std::string& message)
{
    const ros::Time stamp = ros::Time::now();
    const std::string console_msg = "[" + category + "] " + message;
    ROS_INFO("%s", console_msg.c_str());

    if (!enabled_)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!log_stream_.is_open())
    {
        return;
    }

    log_stream_ << std::fixed << std::setprecision(3)
              << stamp.toSec() << " [" << category << "] "
              << message << std::endl;
    log_stream_.flush();
}

} // namespace mapping
} // namespace AVP
