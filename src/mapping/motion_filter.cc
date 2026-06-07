/**
 * @file motion_filter.cc
 * @brief 运动过滤器 - 过滤微小运动，减少冗余子图插入
 *
 * 当车辆移动太小（平移 < 0.2m 且旋转 < 0.2 rad²）时，
 * 跳过子图插入，避免大量重复数据导致子图质量下降和内存浪费。
 */

#include "motion_filter.h"

namespace AVP
{
namespace mapping
{

/**
 * 判断当前位姿是否与上次插入时的位姿太相似
 *
 * @return true = 运动太小，应跳过子图插入
 * @return false = 运动足够大，应该插入子图
 */
bool MotionFilter::IsSimilar(const Eigen::Vector3d& pose)
{
    double translation = std::sqrt(std::pow((pose[0]-last_pose_[0]),2)+std::pow((pose[1]-last_pose_[1]),2));
    double rotation = std::pow((pose[2]-last_pose_[2]),2);
    
    // 阈值: 平移 < 0.2m 且 旋转平方 < 0.2 → 认为位姿没有显著变化
    if (total_ > 0 && translation < 0.2 && rotation < 0.2)
    {
        return true;
    }
    
    // 运动足够大，更新记录并返回 false
    last_pose_ = pose;
    total_++;
    return false;
}

} // namespace mapping
} // namespace AVP
