#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include "ptx/graph_slam_ros.h"
#include "ptx/ros_utils.h"
namespace ptx
{

using ptx::getAndRegisterROSParam;

GraphSlamROS::GraphSlamROS(ros::NodeHandle& n, ros::NodeHandle& np): n_(n), np_(np), tf_listener_(tf_)
{
    tf_.setUsingDedicatedThread(true);

    map_frame_ = getAndRegisterROSParam(np_, "map_frame", "map");
    odom_frame_ = getAndRegisterROSParam(np_, "odom_frame", "odom");
    base_frame_ = getAndRegisterROSParam(np_, "base_frame", "base_footprint");

    poses_pub_ = np_.advertise<geometry_msgs::PoseArray>("poses", 1, true);
    latest_points_pub_ = np_.advertise<sensor_msgs::PointCloud2>("latest_points", 1, true);
    map_points_pub_ = np_.advertise<sensor_msgs::PointCloud2>("map_points", 1, true);
    points_sub_ = n_.subscribe<sensor_msgs::PointCloud2>("points", 1, &GraphSlamROS::pointsCb, this);
    scan_sub_ = n_.subscribe<sensor_msgs::LaserScan>("scan", 1, &GraphSlamROS::scanCb, this);

    map_update_timer_ = n_.createSteadyTimer(ros::WallDuration(2), &GraphSlamROS::mapUpdateTimerCb, this);

    config_ = GraphSlamConfig{};
    slam_ = std::make_shared<GraphSlam>(config_);
}

void GraphSlamROS::pointsCb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    geometry_msgs::PoseStamped robot_pose;
    if (!getRobotPose(odom_frame_, msg->header.stamp, robot_pose))
    {
        return;
    }

    geometry_msgs::TransformStamped base_transform;
    if (!lookupTransform(base_frame_, msg->header.frame_id, msg->header.stamp, base_transform))
    {
        return;
    }

    PointsT msg_1;
    pcl::fromROSMsg(*msg, msg_1);
    PointsT::Ptr msg_in_base_frame = PointsT{}.makeShared();
    pcl::transformPointCloud(msg_1, *msg_in_base_frame, tf2::transformToEigen(base_transform.transform).cast<float>());

    slam_->addNextPose(toPoseT(robot_pose.pose), msg_in_base_frame);

    geometry_msgs::TransformStamped map_to_odom;
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.header.stamp = ros::Time::now();
    map_to_odom.child_frame_id = odom_frame_;
    map_to_odom.transform.translation.x = 0;
    map_to_odom.transform.translation.y = 0;
    map_to_odom.transform.translation.z = 0;
    map_to_odom.transform.rotation.x = 0;
    map_to_odom.transform.rotation.y = 0;
    map_to_odom.transform.rotation.z = 0;
    map_to_odom.transform.rotation.w = 1;
    tf_broadcaster_.sendTransform(map_to_odom);
}

void GraphSlamROS::scanCb(const sensor_msgs::LaserScanConstPtr& msg)
{
    geometry_msgs::PoseStamped robot_pose;
    if (!getRobotPose(odom_frame_, msg->header.stamp, robot_pose))
    {
        return;
    }

    geometry_msgs::TransformStamped base_transform;
    if (!lookupTransform(base_frame_, msg->header.frame_id, msg->header.stamp, base_transform))
    {
        return;
    }

    sensor_msgs::PointCloud2 cloud;
    laser_projection_.projectLaser(*msg, cloud);
    //laser_projection_.transformLaserScanToPointCloud(base_frame_, *msg, cloud, tf_);

    PointsT msg_1;
    pcl::fromROSMsg(cloud, msg_1);
    PointsT::Ptr msg_in_base_frame = PointsT{}.makeShared();
    pcl::transformPointCloud(msg_1, *msg_in_base_frame, tf2::transformToEigen(base_transform.transform).cast<float>());

    slam_->addNextPose(toPoseT(robot_pose.pose), msg_in_base_frame);

    geometry_msgs::TransformStamped map_to_odom;
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.header.stamp = ros::Time::now();
    map_to_odom.child_frame_id = odom_frame_;
    map_to_odom.transform.translation.x = 0;
    map_to_odom.transform.translation.y = 0;
    map_to_odom.transform.translation.z = 0;
    map_to_odom.transform.rotation.x = 0;
    map_to_odom.transform.rotation.y = 0;
    map_to_odom.transform.rotation.z = 0;
    map_to_odom.transform.rotation.w = 1;
    tf_broadcaster_.sendTransform(map_to_odom);
}

void GraphSlamROS::mapUpdateTimerCb(const ros::SteadyTimerEvent& ev)
{
    slam_->updateGlobalMap();
    slam_->optimizeGlobalMap();

    sensor_msgs::PointCloud2 points2;
    pcl::toROSMsg(*slam_->getLocalMapPoints(), points2);
    points2.header.frame_id = map_frame_;
    points2.header.stamp = ros::Time::now();
    map_points_pub_.publish(points2);

    pcl::toROSMsg(*slam_->getGlobalMapPoints(), points2);
    points2.header.frame_id = map_frame_;
    points2.header.stamp = ros::Time::now();
    latest_points_pub_.publish(points2);

    geometry_msgs::PoseArray pose_array;
    for (const auto& pose : slam_->getRegisteredPoses())
    {
        pose_array.poses.emplace_back(fromPoseT<geometry_msgs::Pose>(pose));
    }
    pose_array.header.frame_id = map_frame_;
    pose_array.header.stamp = ros::Time::now();
    poses_pub_.publish(pose_array);
}

}
