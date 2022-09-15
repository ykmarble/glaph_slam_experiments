#include <pcl/registration/gicp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <ptx/ros_utils.h>

#include "ptx/registration_factory.h"

namespace ptx
{

RegistrationT::Ptr GICPFactory(ros::NodeHandle &np)
{
    pcl::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>());
    gicp->setTransformationEpsilon(ptx::getAndRegisterROSParam(np, "reg_transformation_epsilon", 0.01));
    gicp->setMaximumIterations(ptx::getAndRegisterROSParam(np, "reg_maximum_iterations", 64));
    gicp->setUseReciprocalCorrespondences(ptx::getAndRegisterROSParam(np, "reg_use_reciprocal_correspondences", false));
    gicp->setMaxCorrespondenceDistance(ptx::getAndRegisterROSParam(np, "reg_max_correspondence_distance", 2.5));
    gicp->setCorrespondenceRandomness(ptx::getAndRegisterROSParam(np, "reg_correspondence_randomness", 20));
    gicp->setMaximumOptimizerIterations(ptx::getAndRegisterROSParam(np, "reg_max_optimizer_iterations", 20));
    return gicp;
}

RegistrationT::Ptr FastGICPFactory(ros::NodeHandle &np)
{
    fast_gicp::FastGICP<PointT, PointT>::Ptr gicp(new fast_gicp::FastGICP<PointT, PointT>());
    gicp->setNumThreads(ptx::getAndRegisterROSParam(np, "reg_num_threads", 1));
    gicp->setTransformationEpsilon(ptx::getAndRegisterROSParam(np, "reg_transformation_epsilon", 0.01));
    gicp->setMaximumIterations(ptx::getAndRegisterROSParam(np, "reg_maximum_iterations", 64));
    gicp->setMaxCorrespondenceDistance(ptx::getAndRegisterROSParam(np, "reg_max_correspondence_distance", 2.5));
    gicp->setCorrespondenceRandomness(ptx::getAndRegisterROSParam(np, "reg_correspondence_randomness", 20));
    gicp->setRegularizationMethod(fast_gicp::RegularizationMethod::FROBENIUS);
    return gicp;
}

RegistrationT::Ptr FastVGICPFactory(ros::NodeHandle &np)
{
    fast_gicp::FastVGICP<PointT, PointT>::Ptr vgicp(new fast_gicp::FastVGICP<PointT, PointT>());
    vgicp->setNumThreads(ptx::getAndRegisterROSParam(np, "reg_num_threads", 1));
    vgicp->setResolution(ptx::getAndRegisterROSParam(np, "reg_resolution", 1.0));
    vgicp->setTransformationEpsilon(ptx::getAndRegisterROSParam(np, "reg_transformation_epsilon", 0.01));
    vgicp->setMaximumIterations(ptx::getAndRegisterROSParam(np, "reg_maximum_iterations", 64));
    vgicp->setCorrespondenceRandomness(ptx::getAndRegisterROSParam(np, "reg_correspondence_randomness", 20));
    vgicp->setRegularizationMethod(fast_gicp::RegularizationMethod::FROBENIUS);
    return vgicp;
}

static const bool registeredGICP = RegistrationFactory::registerImpl("gicp", &GICPFactory);
static const bool registeredFastGICP = RegistrationFactory::registerImpl("fast_gicp", &FastGICPFactory);
static const bool registeredFastVGICP = RegistrationFactory::registerImpl("fast_vgicp", &FastVGICPFactory);

}
