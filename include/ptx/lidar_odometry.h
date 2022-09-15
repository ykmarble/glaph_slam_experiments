#ifndef LIDAR_ODOMETRY_H_
#define LIDAR_ODOMETRY_H_

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

#include <ptx/ros_utils.h>
#include <ptx/points_insert_criteria_factory.h>
#include <ptx/queued_points_map.h>
#include <ptx/registration_factory.h>
#include <ptx/scan_projector.h>
#include <ptx/types.h>

namespace ptx
{

struct LidarOdometryParameters
{
    std::string lidar_odom_frame;
    std::string odom_frame;
    std::string base_frame;

    std::string registration_method;
    std::string points_insert_criteria;

    double voxel_filter_leaf_size;

    explicit LidarOdometryParameters(ros::NodeHandle& np);
};

class LidarOdometry
{
public:
    LidarOdometry(const ros::NodeHandle& n, const ros::NodeHandle& np);
private:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    LidarOdometryParameters params_;

    tf2_ros::Buffer tf_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    ros::Subscriber scan_sub_;
    ros::Subscriber points_sub_;

    ScanProjector scan_projector_;
    RegistrationT::Ptr registration_;
    QueuedPointsMap local_map_;
    PointsInsertCriteria::Ptr points_insert_criteria_;

    bool local_map_initialized_;
    Eigen::Isometry3f last_odom_to_lio;

    bool getTransform(const std::string& target_frame, const std::string& source_frame, pcl::uint64_t pcl_timestamp, Eigen::Isometry3f& out);
    bool getBasePose(pcl::uint64_t pcl_timestamp, PoseT& out);
    void processPoints(PointsT::Ptr points);  // 'points' will be invalidated. unique_ptr is more suitable.

    void scanCb(const sensor_msgs::LaserScanConstPtr& msg);
    void pointsCb(const sensor_msgs::PointCloud2ConstPtr& msg);
};

}

#endif  //LIDAR_ODOMETRY_H_
