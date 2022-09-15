#ifndef POINT_INSERT_CRITERIA_FACTORY_H_
#define POINT_INSERT_CRITERIA_FACTORY_H_

#include <ros/ros.h>
#include <pcl/registration/registration.h>
#include <ptx/types.h>
#include <ptx/points_map.h>
#include <boost/noncopyable.hpp>

namespace ptx
{

// PointsInsertCriteria is functor that returns "should points to be inserted".
class PointsInsertCriteria
{
public:
    using Ptr = std::unique_ptr<PointsInsertCriteria>;

    // 1st argument is 'robot_pose' in the frame same as 'map', not the relative to the 'map' origin.
    // 2nd argment is 'points' to be inserted, must be registrated to the 'map', meaning the coordinate is relative to the 'map' origin.
    // 3rd argument is 'map', points within the 'map' is relative to the 'map' origin. See PointsMap abstract class.
    //
    // The position relation is a little confusing. In short, 'pose' and 'map' are belong to the same frame.
    // 'points' and points in the 'map' are belong to the same frame ('map' internal representation).
    // If 'map' origin is zero, both frames are the same.
    virtual bool call(const PoseT&, const PointsT&, const PointsMap&) = 0;
};

class PointsInsertCriteriaFactory : boost::noncopyable
{
    using CreateImplT = std::function<PointsInsertCriteria::Ptr(ros::NodeHandle&)>;
public:
    template <typename T>
    static bool registerImpl(const std::string& name)
    {
        static_assert(std::is_base_of<PointsInsertCriteria, T>::value, "T is not a PointsInserCriteria derived class.");

        PointsInsertCriteriaFactory& inst = getInstance();
        const auto result = inst.impl_table_.emplace(name, [](ros::NodeHandle& np){ return std::unique_ptr<T>(new T(np)); });
        if(!result.second)
        {
            throw std::runtime_error("Failed to register PointsInsertCriteriaFactory implementation.");
        }
        return result.second;
    }
    static PointsInsertCriteria::Ptr create(const std::string& name, ros::NodeHandle& np);
private:
    std::map<std::string, CreateImplT> impl_table_;  // Must be singleton member, otherwise the initialization order is undefined.

    PointsInsertCriteriaFactory() = default;
    static PointsInsertCriteriaFactory& getInstance();
};

}

#endif  //POINT_INSERT_CRITERIA_FACTORY_H_
