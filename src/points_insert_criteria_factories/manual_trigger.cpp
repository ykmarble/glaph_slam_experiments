#include "ptx/points_insert_criteria_factory.h"
#include <std_srvs/Empty.h>
#include <atomic>

namespace ptx
{

class ManualTriggerCriteria : public PointsInsertCriteria
{
public:
    explicit ManualTriggerCriteria(ros::NodeHandle& np) : should_update_(false)
    {
        trigger_srv_ = np.advertiseService("insert_points_trigger", &ManualTriggerCriteria::triggerCb, this);
    }
    bool call(const ptx::PoseT& pose, const ptx::PointsT&, const ptx::PointsMap&)
    {
        if (should_update_)
        {
            should_update_ = false;
            return true;
        }
        else
        {
            return false;
        }
    }
private:
    ros::ServiceServer trigger_srv_;
    bool should_update_;
    bool triggerCb(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
    {
        should_update_ = true;
        return true;
    }
};

static const bool registeredManualTrigger = PointsInsertCriteriaFactory::registerImpl<ManualTriggerCriteria>("manual_trigger");

}
