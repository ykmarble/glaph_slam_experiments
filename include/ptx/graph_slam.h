#ifndef GRAPH_SLAM_H_
#define GRAPH_SLAM_H_

#include "ptx/keyframe.h"
#include <iostream>
#include <deque>
#include <mutex>
#include <boost/noncopyable.hpp>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

namespace ptx
{

struct KeyFrameVertexPair
{
    std::shared_ptr<KeyFrame> keyframe;
    //std::shared_ptr<PoseVertexT> vertex;
    PoseVertexT* vertex;
};

struct GraphSlamConfig
{
    size_t local_map_queue_size;
    std::string local_map_registration_method;
    double voxel_leaf_size_xy;
    double voxel_leaf_size_z;
    size_t graph_optimization_steps;

    GraphSlamConfig():
        local_map_queue_size(10),
        local_map_registration_method("gicp"),
        voxel_leaf_size_xy(0.05),
        voxel_leaf_size_z(0.1),
        graph_optimization_steps(1024)
    {}

    GraphSlamConfig& setLocalMapRegistrationMethod(const std::string& method)
    {
        local_map_registration_method = method;
        return *this;
    }
};

class GraphSlam : private boost::noncopyable
{
public:
    explicit GraphSlam(const GraphSlamConfig config);
    void addNextPose(const PoseT& next_pose, const PointsT::Ptr& points);
    void updateGlobalMap();
    void optimizeGlobalMap();
    std::vector<PoseT> getRegisteredPoses() const;
    PointsT::Ptr getLocalMapPoints() const;
    PointsT::Ptr getGlobalMapPoints() const;

private:
    using MutexT = std::recursive_mutex;
    using SlamBlockSolver = g2o::BlockSolverX;
    using SlamLinearSolver = g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;
    using Registration = pcl::Registration<PointsT, PointsT>;

    mutable MutexT mtx_;

    GraphSlamConfig config_;

    std::deque<KeyFrame::Ptr> local_map_;
    std::vector<KeyFrame::Ptr> global_map_;
    PointsT::Ptr global_map_cloud_;
    GraphT graph_;

    int next_id_;
    std::map<int, KeyFrameVertexPair> id_table_;

    //std::shared_ptr<PoseVertexT> fixed_vertex_;
    PoseVertexT* fixed_vertex_;

    void registerVertex();
    PointsT::Ptr toGlobalPoints(const KeyFrame& keyframe) const;
};

}

#endif  //GRAPH_SLAM_H_
