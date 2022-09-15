#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "ptx/graph_slam_ros.h"

namespace ptx
{

class GlaphSlamROSNodelet : public nodelet::Nodelet
{
public:
    GlaphSlamROSNodelet() {}
    ~GlaphSlamROSNodelet() {}

private:
    virtual void onInit()
    {
        lp_.reset(new GraphSlamROS(getNodeHandle(), getPrivateNodeHandle()));
    }

    boost::shared_ptr<GraphSlamROS> lp_;
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ptx::GlaphSlamROSNodelet, nodelet::Nodelet);
