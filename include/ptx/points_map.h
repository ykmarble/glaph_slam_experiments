#ifndef POINT_CLOUD_MAP_H_
#define POINT_CLOUD_MAP_H_

#include <boost/noncopyable.hpp>
#include <ptx/types.h>

namespace ptx
{

class PointsMap : boost::noncopyable
{
public:
    // Add points to the map. 'points' must be relative to the 'origin'.
    virtual void addPoints(const PointsT::ConstPtr& points) = 0;
    virtual void addPoints(PointsT&& points) = 0;

    // Return map point cloud relateve to the origin.
    virtual PointsT::ConstPtr map() const = 0;

    std::string frame_id;
    PoseT origin;
};

}

#endif  //POINT_CLOUD_MAP_H_
