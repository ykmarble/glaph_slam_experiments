#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include "ptx/scan_projector.h"

class Scan2CloudNode
{
public:
    Scan2CloudNode(const ros::NodeHandle& n, const ros::NodeHandle& np): n_(n), np_(np)
    {
        pub_ = ros::NodeHandle().advertise<sensor_msgs::PointCloud2>("scan_cloud", 10);
        sub_ = ros::NodeHandle().subscribe("scan", 10, &Scan2CloudNode::scanCb, this);
    }

private:
    ros::NodeHandle n_;
    ros::NodeHandle np_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ptx::ScanProjector proj_;

    void scanCb(const sensor_msgs::LaserScanConstPtr& msg)
    {
        pub_.publish(proj_(*msg));
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan2cloud_node");
    auto scn = Scan2CloudNode(ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();
    return 0;
}
