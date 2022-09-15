#include "ptx/lidar_odometry.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <ptx/ros_adaptor.h>


struct DebugPublishers
{
    ros::Publisher points_from_scan;
    ros::Publisher local_map;
    ros::Publisher registration_result;

    void advertise(ros::NodeHandle& np)
    {
        points_from_scan = np.advertise<sensor_msgs::PointCloud2>("points_from_scan", 1, true);
        local_map = np.advertise<sensor_msgs::PointCloud2>("local_map", 1, true);
        registration_result = np.advertise<sensor_msgs::PointCloud2>("registration_result", 1, true);
    }

    void publish_registration_result(const std::string& frame_id, const ptx::PointsT& src,  const ptx::PointsT& tgt, const ptx::PointsT& result)
    {
        pcl::PointCloud<pcl::PointXYZRGB> msg;
        msg.header.frame_id = frame_id;
        msg.reserve(src.size() + tgt.size()+ result.size());
        for (const auto& p : src)
        {
            msg.push_back(pcl::PointXYZRGB{255, 0, 0});
            msg.back().getVector3fMap() = p.getVector3fMap();
        }
        for (const auto& p : tgt)
        {
            msg.push_back(pcl::PointXYZRGB{255, 255, 255});
            msg.back().getVector3fMap() = p.getVector3fMap();
        }
        for (const auto& p : result)
        {
            msg.push_back(pcl::PointXYZRGB{0, 255, 0});
            msg.back().getVector3fMap() = p.getVector3fMap();
        }
        registration_result.publish(msg);
    }

} debug_pub;

void assert_cloud(const ptx::PointsT& points)
{
    for (const auto& p : points)
    {
        if (!std::isfinite(p.x)) throw std::runtime_error("Invalid point found. (x)");
        if (!std::isfinite(p.y)) throw std::runtime_error("Invalid point found. (y)");
        if (!std::isfinite(p.z)) throw std::runtime_error("Invalid point found. (z)");
        if (!std::isfinite(p.intensity)) throw std::runtime_error("Invalid point found. (intensity)");
    }
}


ptx::PointsT::Ptr voxel_filter(ptx::PointsT::ConstPtr in, double leaf_size)
{
    pcl::VoxelGrid<ptx::PointT> vg;
    vg.setInputCloud(in);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    pcl::PointCloud<ptx::PointT>::Ptr out(new pcl::PointCloud<ptx::PointT>());
    vg.filter(*out);
    return out;
}


namespace ptx
{

LidarOdometryParameters::LidarOdometryParameters(ros::NodeHandle& np)
{
    lidar_odom_frame = ptx::getAndRegisterROSParam(np, "lidar_odom_frame", "lidar_odom");
    odom_frame = ptx::getAndRegisterROSParam(np, "odom_frame", "odom");
    base_frame = ptx::getAndRegisterROSParam(np, "base_frame", "base_footprint");
    registration_method = ptx::getAndRegisterROSParam(np, "registration_method", "fast_vgicp");
    points_insert_criteria = ptx::getAndRegisterROSParam(np, "points_insert_criteria", "traveled_distance");
    voxel_filter_leaf_size = ptx::getAndRegisterROSParam(np, "voxel_filter_leaf_size", 0.05);
}

LidarOdometry::LidarOdometry(const ros::NodeHandle& n, const ros::NodeHandle& np):
    n_(n),
    np_(np),
    params_(np_),
    tf_listener_(tf_),
    scan_sub_(n_.subscribe("scan", 10, &LidarOdometry::scanCb, this)),
    points_sub_(n_.subscribe("points", 10, &LidarOdometry::pointsCb, this)),
    registration_(RegistrationFactory::create(params_.registration_method, np_)),
    local_map_(n, np_),
    points_insert_criteria_(PointsInsertCriteriaFactory::create(params_.points_insert_criteria, np_)),
    local_map_initialized_(false),
    last_odom_to_lio(Eigen::Isometry3f::Identity())
{
    local_map_.frame_id = params_.lidar_odom_frame;
    debug_pub.advertise(np_);
}

bool LidarOdometry::getTransform(const std::string& target_frame, const std::string& source_frame, pcl::uint64_t pcl_timestamp, Eigen::Isometry3f& out)
{
    try
    {
        const auto transform_msg = tf_.lookupTransform(target_frame, source_frame, pcl_conversions::fromPCL(pcl_timestamp), ros::Duration(0.1));
        out = tf2::transformToEigen(transform_msg.transform).cast<float>();
    }
    catch (const tf2::TransformException& e)
    {
        ROS_WARN("Transform between '%s' and '%s' unavailable.", source_frame.c_str(), target_frame.c_str());
        return false;
    }
    return true;
}

bool LidarOdometry::getBasePose(pcl::uint64_t pcl_timestamp, PoseT& out)
{
    Eigen::Isometry3f out_3d;
    if (getTransform(params_.odom_frame, params_.base_frame, pcl_timestamp, out_3d))
    {
        // The most stupid part. I can't extract yaw from Eigen::Quaternion directly.
        Eigen::Quaternionf qe{out_3d.rotation()};
        out = Eigen::Translation2d{out_3d.translation().head<2>().cast<double>()}
                * Eigen::Rotation2Dd{tf2::getYaw(tf2::Quaternion{qe.x(), qe.y(), qe.z(), qe.w()})};
        return true;
    }
    else
    {
        return false;
    }
}

void LidarOdometry::processPoints(PointsT::Ptr points)
{
    assert_cloud(*points);
    auto points_filtered = voxel_filter(points, params_.voxel_filter_leaf_size);

    Eigen::Isometry3f sensor_to_odom;
    if (!getTransform(params_.odom_frame, points->header.frame_id, points->header.stamp, sensor_to_odom))
    {
        return;
    }

    // TODO: Support local_map origin. Current implementation assumes identity transform.
    //Eigen::Isometry3f lio_to_local_map = Eigen::Isometry3f::Identity();
    //lio_to_local_map.translation().head<2>() = local_map_.origin.translation().cast<float>();
    //lio_to_local_map.linear() = Eigen::AngleAxisf{Eigen::Rotation2Df{local_map_.origin.rotation()}.angle(), Eigen::Vector3f::UnitZ()}.matrix();

    PointsT::Ptr points_guess(new PointsT{});
    pcl::transformPointCloud(*points_filtered, *points_guess, last_odom_to_lio * sensor_to_odom);

    if (!local_map_initialized_)
    {
        local_map_.addPoints(points_guess);
        debug_pub.local_map.publish(local_map_.map());
        registration_->setInputTarget(local_map_.map());
        local_map_initialized_ = true;
        return;
    }

    PointsT::Ptr points_aligned(new PointsT{});
    registration_->setInputSource(points_guess);
    registration_->align(*points_aligned);
    if (!registration_->hasConverged())
    {
        ROS_WARN("!!!! NOT CONVERGED !!!!");
        return;
    }
    Eigen::Isometry3f trans(registration_->getFinalTransformation().cast<float>());
    double dx = trans.matrix().block<3, 1>(0, 3).norm();
    double da = std::acos(Eigen::Quaternionf(trans.matrix().block<3, 3>(0, 0)).w());
    if (dx > 0.5 || da > M_PI * 5. / 180)
    {
        ROS_WARN("!!!! DETECT JUMP !!!!");
        return;
    }
    last_odom_to_lio = trans * last_odom_to_lio;
    debug_pub.publish_registration_result(params_.lidar_odom_frame, *points_guess, *local_map_.map(), *points_aligned);

    geometry_msgs::TransformStamped transform_msg = tf2::eigenToTransform(last_odom_to_lio.cast<double>());
    transform_msg.header.frame_id = params_.lidar_odom_frame;
    transform_msg.header.stamp = pcl_conversions::fromPCL(points->header.stamp);
    transform_msg.child_frame_id = params_.odom_frame;
    tf_broadcaster_.sendTransform(transform_msg);

    PoseT estimated_pose;
    if (!getBasePose(points->header.stamp, estimated_pose))
    {
        return;
    }

    if (points_insert_criteria_->call(estimated_pose, *points_aligned, local_map_))
    {
        ROS_INFO("Inserting new points.");
        local_map_.addPoints(points_aligned);
        debug_pub.local_map.publish(local_map_.map());
        // fast_gicp seems inheriting the implicit status about matching. Respawn the instance.
        registration_ = RegistrationFactory::create(params_.registration_method, np_);
        auto local_map_filtered = voxel_filter(local_map_.map(), params_.voxel_filter_leaf_size);
        registration_->setInputTarget(local_map_filtered);
    }
}

void LidarOdometry::scanCb(const sensor_msgs::LaserScanConstPtr& msg)
{
    PointsT::Ptr points(new PointsT{});
    *points = scan_projector_(*msg);
    debug_pub.points_from_scan.publish(*points);
    processPoints(points);
}

void LidarOdometry::pointsCb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    PointsT::Ptr points(new PointsT{});
    pcl::fromROSMsg(*msg, *points);
    processPoints(points);
}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_odometry");
    ros::NodeHandle n;
    ros::NodeHandle np("~");
    ptx::LidarOdometry lo(n, np);
    ros::spin();
    return 0;
}
