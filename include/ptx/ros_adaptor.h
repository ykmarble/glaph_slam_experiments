#ifndef ROS_ADAPTOR_H_
#define ROS_ADAPTOR_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "ptx/types.h"

namespace ptx
{
inline PoseT toPoseT(const geometry_msgs::Pose& p)
{
    return Eigen::Translation2d(p.position.x, p.position.y) * Eigen::Rotation2Dd(tf2::getYaw(p.orientation));
}

inline TimeT toTimeT(const ros::Time& t)
{
    return TimeT() + std::chrono::nanoseconds(t.toNSec());
}

template <typename outT> inline outT fromPoseT(const PoseT&);

template <>
inline geometry_msgs::Pose fromPoseT(const PoseT& p)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, Eigen::Rotation2Dd(p.rotation()).angle());
    geometry_msgs::Pose p_out;
    p_out.position.x = p.translation().x();
    p_out.position.y = p.translation().y();
    p_out.position.z = 0; // Assign explicitly because p.translation().z() returns 1.
    p_out.orientation = tf2::toMsg(q);
    return p_out;
}


template <typename outT> inline outT fromTimeT(const TimeT&);

template <>
inline ros::Time fromTimeT(const TimeT& t)
{
    return ros::Time(0, std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count());
}

}

#endif  //ROS_ADAPTOR_H_
