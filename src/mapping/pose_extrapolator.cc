#include "pose_extrapolator.h"

namespace AVP
{
namespace mapping
{

    void PoseExtrapolator::AddPose(const Eigen::Vector3d& pose)
    {   
        std::unique_lock<std::mutex> lock(mMutexQueue);
        timed_pose_queue_.push_back(pose);
        if (odometry_pose_queue_.empty())
        {
            last_odometry_pose_[0] = 0.f;
            last_odometry_pose_[1] = 0.f;
            last_odometry_pose_[2] = 0.f;
        }
        else
        {
            last_odometry_pose_[0] = odometry_pose_queue_.back()[0];
            last_odometry_pose_[1] = odometry_pose_queue_.back()[1];
            last_odometry_pose_[2] = odometry_pose_queue_.back()[2];
        }
    }

    void PoseExtrapolator::AddOdometry(const Eigen::Vector3d& odometry_pose)
    {   
        std::unique_lock<std::mutex> lock(mMutexQueue);   
        odometry_pose_queue_.push_back(odometry_pose);
    }

    /**
     * @brief 预测当前位姿
     * @return 预测的位姿 (x, y, theta)
     * 
     * 基于里程计的增量运动来预测当前位姿，算法原理：
     * predicted_pose = last_corrected_pose + (current_odometry - last_odometry)
     * 
     * 这个方法用于为扫描匹配提供初始位姿估计，提高匹配的收敛速度和准确性
     */
    Eigen::Vector3d PoseExtrapolator::PredictPose()
    { 
        // 线程安全：获取互斥锁，防止多线程同时访问队列
        std::unique_lock<std::mutex> lock(mMutexQueue);   

        // 获取最新的里程计位姿
        Eigen::Vector3d current_odometry_pose = odometry_pose_queue_.back();
        
        // 获取最后一次经过扫描匹配校正的位姿
        Eigen::Vector3d last_stored_pose = timed_pose_queue_.back(); 

        // 计算预测位姿
        Eigen::Vector3d ret;
        // 位姿预测公式：当前校正位姿 + 里程计增量
        // 里程计增量 = 当前里程计位姿 - 上次记录的里程计位姿
        ret = last_stored_pose + (current_odometry_pose - last_odometry_pose_);

        return ret;
    }

} // namespace mapping
} // namespace AVP
