#ifndef QUEUED_POINTS_MAP_H_
#define QUEUED_POINTS_MAP_H_

#include <ros/ros.h>
#include <pcl/common/common.h>
#include <ptx/types.h>
#include <ptx/ros_utils.h>
#include <ptx/points_map.h>

namespace ptx
{

struct QueuedPointsMapParameters
{
    int queue_size;
    explicit QueuedPointsMapParameters(ros::NodeHandle& np);
};

class QueuedPointsMap : public PointsMap
{
public:
    QueuedPointsMap(const ros::NodeHandle& n, const ros::NodeHandle& np);
    void addPoints(const PointsT::ConstPtr& points) override;
    void addPoints(PointsT&& points) override;
    PointsT::ConstPtr map() const override;

private:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    QueuedPointsMapParameters params_;

    int oldest_points_index_;
    std::vector<PointsT::Ptr> points_queue_;
    PointsT::Ptr merged_cloud_;

    void rebuildCloud();
};

}

#endif  //QUEUED_POINTS_MAP_H_
