#include "ptx/points_insert_criteria_factory.h"
#include <ptx/ros_utils.h>

namespace ptx
{

class TraveledDistanceCriteria : public PointsInsertCriteria
{
public:
    explicit TraveledDistanceCriteria(ros::NodeHandle& np)
    {
        threshold_2_ = std::pow(ptx::getAndRegisterROSParam(np, "distance_threshold", 0.5), 2.);
    }
    bool call(const ptx::PoseT& pose, const ptx::PointsT&, const ptx::PointsMap&)
    {
        bool should_insert = (pose.translation() - last_pose_.translation()).squaredNorm() > threshold_2_;
        if (should_insert)
        {
            last_pose_ = pose;
        }
        return should_insert;
    }
private:
    double threshold_2_;
    ptx::PoseT last_pose_;
};

static const bool registeredTraveledDistance = PointsInsertCriteriaFactory::registerImpl<TraveledDistanceCriteria>("traveled_distance");

}
