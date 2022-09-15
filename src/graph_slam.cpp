#include <g2o/core/optimization_algorithm_levenberg.h>
#include "ptx/log.h"
#include "ptx/graph_slam.h"
#include <pcl/filters/voxel_grid.h>

namespace ptx
{

GraphSlam::GraphSlam(const GraphSlamConfig config): next_id_(0), config_(config)
{
    LOG_INFO("Initializing GraphSlam.");
    graph_.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg{new SlamBlockSolver{new SlamLinearSolver{}}});
    global_map_cloud_ = PointsT{}.makeShared();

    // Fixed vertex is necessary to optimize.
    fixed_vertex_ = new PoseVertexT();
    fixed_vertex_->setId(next_id_);
    fixed_vertex_->setEstimate(PoseT::Identity());
    fixed_vertex_->setFixed(true);
    graph_.addVertex(fixed_vertex_);
    next_id_++;
}

void GraphSlam::addNextPose(const PoseT& next_pose, const PointsT::Ptr& points)
{
    std::lock_guard<MutexT> lk(mtx_);

    // Crate new keyframe
    auto kf = std::make_shared<KeyFrame>();
    kf->stamp = std::chrono::system_clock::time_point();  // TODO: Set from the observation.
    kf->id = -1;
    kf->pose = next_pose;
    kf->points = points;
    local_map_.push_back(kf);
    while (local_map_.size() > config_.local_map_queue_size) {
        local_map_.pop_front();
    }

    // TODO: use lidar odometry

}

void GraphSlam::updateGlobalMap()
{
    std::lock_guard<MutexT> lk(mtx_);

    if (local_map_.empty())
    {
        return;
    }

    // TODO: Pick up and gather good keyframes

    for (const auto& kf : local_map_)
    {
        // Create new graph vertex
        auto v = new PoseVertexT();
        v->setId(next_id_);
        v->setEstimate(kf->pose);
        kf->id = next_id_;

        // Store pose informations
        graph_.addVertex(v);
        id_table_.emplace(next_id_, KeyFrameVertexPair{kf, v});

        // TODO: Calculate stddev?
        const double stddev_x = 0.2;
        const double stddev_th = 0.2;

        // Create information matrix
        Eigen::MatrixXd info_mat = Eigen::MatrixXd::Identity(6, 6);
        info_mat.topLeftCorner(3, 3).array() /= stddev_x;
        info_mat.bottomRightCorner(3, 3).array() /= stddev_th;

        // Create new graph edge
        auto e = new PoseEdgeT();
        e->setInformation(info_mat);
        if (global_map_.empty())
        {
            e->setMeasurement(kf->pose.inverse());
            e->vertices()[0] = fixed_vertex_;
        }
        else
        {
            e->setMeasurement(kf->pose.inverse() * global_map_.back()->pose);
            e->vertices()[0] = id_table_[global_map_.back()->id].vertex;
        }
        e->vertices()[1] = id_table_[kf->id].vertex;
        graph_.addEdge(e);

        // Update pose inserter status
        next_id_++;
    }

    auto cache_cloud = PointsT{}.makeShared();
    for (const auto& kf : global_map_)
    {
        *cache_cloud += *toGlobalPoints(*kf);
    }

    // global map down sizing
    pcl::VoxelGrid<PointsT::PointType> vg;
    vg.setInputCloud(cache_cloud);
    vg.setLeafSize(config_.voxel_leaf_size_xy, config_.voxel_leaf_size_xy, config_.voxel_leaf_size_z);
    vg.filter(*global_map_cloud_);

}

void GraphSlam::optimizeGlobalMap()
{
    std::lock_guard<MutexT> lk(mtx_);

    // optimize pose graph
    graph_.initializeOptimization();
    graph_.setVerbose(true);
    graph_.optimize(config_.graph_optimization_steps);
}

PointsT::Ptr GraphSlam::toGlobalPoints(const KeyFrame& kf) const
{
    auto cloud = PointsT{}.makeShared();
    PointsT tmp_cloud;
    Eigen::Vector3f t(kf.pose.translation().x(), kf.pose.translation().y(), 0);
    Eigen::Quaternionf q(Eigen::AngleAxisf(Eigen::Rotation2Df(kf.pose.rotation()).angle(), Eigen::Vector3f::UnitZ()));

    pcl::transformPointCloud(*kf.points, tmp_cloud, t, q);
    *cloud += tmp_cloud;
    return cloud;
}

std::vector<PoseT> GraphSlam::getRegisteredPoses() const
{
    std::lock_guard<MutexT> lk(mtx_);

    std::vector<PoseT> out;
    for (const auto& kf: local_map_)
    {
        out.push_back(kf->pose);
    }
    return out;
}

PointsT::Ptr GraphSlam::getLocalMapPoints() const
{
    std::lock_guard<MutexT> lk(mtx_);

    PointsT::Ptr out = PointsT{}.makeShared();
    for (const auto& kf : local_map_)
    {
        *out += *toGlobalPoints(*kf);
    }
    return out;
}

PointsT::Ptr GraphSlam::getGlobalMapPoints() const
{
    std::lock_guard<MutexT> lk(mtx_);

    PointsT::Ptr out = PointsT{}.makeShared();
    *out = *global_map_cloud_;
    return out;
}

}
