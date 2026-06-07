/**
 * @file pose_extrapolator.cc
 * @brief 位姿外推器 - 利用里程计增量预测当前位姿
 *
 * 核心思想：
 *   将上一次扫描匹配校正后的位姿 + 里程计的增量，预测当前位姿。
 *   这个预测值会作为扫描匹配的初始值。
 *
 *   预测公式：
 *     predicted_pose = last_corrected_pose + (current_odom - last_odom_at_correction_time)
 *
 *   时间线：
 *     t0: 扫描匹配校正位姿 P0 → AddPose(P0)，记住此时的里程计 O0
 *     t1: 里程计更新 O1 → AddOdometry(O1)
 *     t2: 新语义扫描到来 → PredictPose() = P0 + (O1 - O0)
 *     t3: 扫描匹配校正位姿 P1 → AddPose(P1)，记住此时的里程计
 *     ...
 *
 *   注意：这里没有用时间戳做插值，是简化版本
 */

#include "pose_extrapolator.h"

namespace AVP
{
namespace mapping
{

    /**
     * 添加扫描匹配校正后的位姿
     *
     * 同时记录此时的里程计值（last_odometry_pose_），
     * 下次预测时用里程计增量叠加到这个校正位姿上
     */
    void PoseExtrapolator::AddPose(const Eigen::Vector3d& pose)
    {   
        std::unique_lock<std::mutex> lock(mMutexQueue);
        timed_pose_queue_.push_back(pose);

        // 记住当前的里程计值，作为后续计算里程计增量的基准
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

    // 接收里程计数据，加入队列
    void PoseExtrapolator::AddOdometry(const Eigen::Vector3d& odometry_pose)
    {   
        std::unique_lock<std::mutex> lock(mMutexQueue);   
        odometry_pose_queue_.push_back(odometry_pose);
    }

    /**
     * 预测当前位姿
     *
     * 公式: predicted = last_corrected_pose + (current_odom - odom_at_last_correction)
     * 即：上次校正后的位姿 + 自那以后里程计的增量
     */
    Eigen::Vector3d PoseExtrapolator::PredictPose()
    { 
        std::unique_lock<std::mutex> lock(mMutexQueue);   

        Eigen::Vector3d current_odometry_pose = odometry_pose_queue_.back();
        Eigen::Vector3d last_stored_pose = timed_pose_queue_.back(); 

        Eigen::Vector3d ret;
        ret = last_stored_pose + (current_odometry_pose - last_odometry_pose_);

        return ret;
    }

} // namespace mapping
} // namespace AVP
