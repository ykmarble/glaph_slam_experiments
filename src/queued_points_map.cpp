#include "ptx/queued_points_map.h"

namespace ptx
{

QueuedPointsMapParameters::QueuedPointsMapParameters(ros::NodeHandle& np)
{
    queue_size = ptx::getAndRegisterROSParam(np, "queue_size", 100);
}

QueuedPointsMap::QueuedPointsMap(const ros::NodeHandle& n, const ros::NodeHandle& np):
    n_(n),
    np_(np),
    params_(np_),
    oldest_points_index_(0)
{
    origin = PoseT::Identity();
    points_queue_.resize(params_.queue_size);
    for (auto& p: points_queue_)
    {
        p.reset(new PointsT{});  // Initialize with valid pointers for similicity.
    }

    merged_cloud_.reset(new PointsT{});
}

void QueuedPointsMap::addPoints(const PointsT::ConstPtr& points)
{
    assert(points);
    *points_queue_[oldest_points_index_++] = *points;
    if (oldest_points_index_ >= points_queue_.size())
    {
        oldest_points_index_ = 0;
    }
    rebuildCloud();
}

void QueuedPointsMap::addPoints(PointsT&& points)
{
    points_queue_[oldest_points_index_++]->swap(points);
    if (oldest_points_index_ >= points_queue_.size())
    {
        oldest_points_index_ = 0;
    }
    rebuildCloud();
}

PointsT::ConstPtr QueuedPointsMap::map() const
{
    return merged_cloud_;
}

void QueuedPointsMap::rebuildCloud()
{
    merged_cloud_->clear();
    merged_cloud_->header.frame_id = frame_id;
    size_t cloud_size = 0;
    for (const auto& i : points_queue_)
    {
        cloud_size += i->size();
    }
    merged_cloud_->reserve(cloud_size);
    for (const auto& i : points_queue_)
    {
        *merged_cloud_ += *i;
    }
}

}
