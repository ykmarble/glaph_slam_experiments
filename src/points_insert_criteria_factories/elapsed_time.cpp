#include "ptx/points_insert_criteria_factory.h"
#include <ptx/ros_utils.h>
#include <chrono>

namespace ptx
{

class ElapsedTimeCriteria : public PointsInsertCriteria
{
public:
    explicit ElapsedTimeCriteria(ros::NodeHandle& np)
    {
        double span_sec = ptx::getAndRegisterROSParam(np, "update_span", 1.);
        span_ = std::chrono::milliseconds(static_cast<std::chrono::milliseconds::rep>(span_sec * 1000));
        next_update_ = std::chrono::steady_clock::now();
    }
    bool call(const ptx::PoseT& pose, const ptx::PointsT&, const ptx::PointsMap&)
    {
        const auto now = std::chrono::steady_clock::now();
        bool should_update = now > next_update_;
        if (should_update)
        {
            next_update_ = now + span_;
        }
        return should_update;
    }
private:
    std::chrono::milliseconds span_;
    std::chrono::steady_clock::time_point next_update_;
};

static const bool registeredElapsedTime = PointsInsertCriteriaFactory::registerImpl<ElapsedTimeCriteria>("elapsed_time");

}
