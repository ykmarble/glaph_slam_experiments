#ifndef TYPES_H_
#define TYPES_H_

#include <chrono>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/core/sparse_optimizer.h>

namespace ptx
{

using PoseT = Eigen::Isometry2d;
using TimeT = std::chrono::system_clock::time_point;
using PointT = pcl::PointXYZI;
using PointsT = pcl::PointCloud<PointT>;
using RegistrationT = pcl::Registration<PointT, PointT>;
using GraphT = g2o::SparseOptimizer;
using PoseVertexT = g2o::VertexSE2;
using PoseEdgeT = g2o::EdgeSE2;

using PoseVertexStore = std::vector<std::shared_ptr<PoseVertexT>>;
using PoseEdgeStore = std::vector<std::shared_ptr<PoseEdgeT>>;

}

#endif  //TYPES_H_
