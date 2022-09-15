#include "ptx/points_insert_criteria_factory.h"

namespace ptx
{

PointsInsertCriteriaFactory& PointsInsertCriteriaFactory::getInstance()
{
    static PointsInsertCriteriaFactory inst;
    return inst;
}

PointsInsertCriteria::Ptr PointsInsertCriteriaFactory::create(const std::string& name, ros::NodeHandle& np)
{
    PointsInsertCriteriaFactory& inst = getInstance();
    const auto impl = inst.impl_table_.find(name);
    if (impl == inst.impl_table_.end())
    {
        throw std::runtime_error("Unknown PointsInsertCriteria " + name);
    }
    ROS_INFO("Created %s PointsInsertCriteria instance.", name.c_str());
    return impl->second(np);
}

}
