#ifndef KEYFRAME_H_
#define KEYFRAME_H_

#include <map>

#include "ptx/types.h"

namespace ptx
{

struct KeyFrame
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<KeyFrame>;
    using ConstPtr = std::shared_ptr<const KeyFrame>;

    int id;
    TimeT stamp;
    PoseT pose;
    PointsT::Ptr points;

    void save(const std::string& path);
    static KeyFrame::Ptr load(const std::string& path);
};

}

#endif  //KEYFRAME_H_
