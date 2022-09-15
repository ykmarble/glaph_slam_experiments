#ifndef GRAPH_SLAM_ROS_H_
#define GRAPH_SLAM_ROS_H_

#include "ptx/graph_slam.h"
#include "ptx/ros_adaptor.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

namespace ptx
{

class GraphSlamROS
{
public:
    GraphSlamROS(ros::NodeHandle& n, ros::NodeHandle& np);
private:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    ros::Publisher poses_pub_;
    ros::Publisher latest_points_pub_;
    ros::Publisher map_points_pub_;
    ros::Subscriber points_sub_;
    ros::Subscriber scan_sub_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    ros::SteadyTimer map_update_timer_;

    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;

    tf2_ros::Buffer tf_;
    laser_geometry::LaserProjection laser_projection_;
    std::shared_ptr<GraphSlam> slam_;
    GraphSlamConfig config_;

    void pointsCb(const sensor_msgs::PointCloud2ConstPtr& msg);
    void scanCb(const sensor_msgs::LaserScanConstPtr& msg);
    void mapUpdateTimerCb(const ros::SteadyTimerEvent& ev);

    bool lookupTransform(const std::string& to, const std::string& from, const ros::Time& t, geometry_msgs::TransformStamped& out)
    {
        try
        {
            out = tf_.lookupTransform(to, from, t, ros::Duration(0.1));
        }
        catch (const tf2::TransformException& e)
        {
            ROS_WARN("Transform between '%s' and '%s' unavailable.", to.c_str(), from.c_str());
            return false;
        }
        return true;
    }

    bool getRobotPose(const std::string& global_frame, const ros::Time& t, geometry_msgs::PoseStamped& out)
    {
        geometry_msgs::PoseStamped identity_pose;
        identity_pose.header.frame_id = base_frame_;
        identity_pose.header.stamp = t;
        identity_pose.pose.position.x = 0;
        identity_pose.pose.position.y = 0;
        identity_pose.pose.position.z = 0;
        identity_pose.pose.orientation.x = 0;
        identity_pose.pose.orientation.y = 0;
        identity_pose.pose.orientation.z = 0;
        identity_pose.pose.orientation.w = 1;
        try
        {
            out = tf_.transform(identity_pose, global_frame, ros::Duration(0.1));
        }
        catch (const tf2::TransformException& e)
        {
            ROS_WARN("Transform between '%s' and '%s' unavailable.", base_frame_.c_str(), global_frame.c_str());
            return false;
        }
        return true;
    }
};

}

#endif  //GRAPH_SLAM_ROS_H_
