#ifndef ROS_UTILS_H_
#define ROS_UTILS_H_

#include <ros/ros.h>

namespace ptx
{

template <typename T>
T getAndRegisterROSParam(
    ros::NodeHandle& np,
    const std::string& param_name,
    const T& default_val)
{
    const auto p = np.param(param_name, default_val);
    np.setParam(param_name, p);
    return p;
}

inline std::string getAndRegisterROSParam(
    ros::NodeHandle& np,
    const std::string& param_name,
    const char* default_val)
{
    const auto p = np.param(param_name, std::string(default_val));
    np.setParam(param_name, p);
    return p;
}

}


#endif  //ROS_UTILS_H_
