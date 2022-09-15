#include <ros/ros.h>
#include "ptx/graph_slam_ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "graph_slam_ros_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");
    ptx::GraphSlamROS slam(n, np);
    ros::spin();
    return 0;
}
