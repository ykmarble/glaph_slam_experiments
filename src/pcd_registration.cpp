#include <chrono>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/gicp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr GICPFactory()
{
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr gicp(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
    gicp->setTransformationEpsilon(0.01);
    gicp->setMaximumIterations(64);
    gicp->setUseReciprocalCorrespondences(false);
    gicp->setMaxCorrespondenceDistance(2.5);
    gicp->setCorrespondenceRandomness(20);
    gicp->setMaximumOptimizerIterations(20);
    return gicp;
}

pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr FastGICPFactory(int method)
{
    fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>::Ptr gicp(new fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>());
    gicp->setNumThreads(1);
    gicp->setTransformationEpsilon(0.01);
    gicp->setMaximumIterations(64);
    gicp->setMaxCorrespondenceDistance(2.5);
    gicp->setCorrespondenceRandomness(20);
    gicp->setRegularizationMethod(static_cast<fast_gicp::RegularizationMethod>(method));
    return gicp;
}

pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr FastVGICPFactory(int method)
{
    fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ>::Ptr vgicp(new fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ>());
    vgicp->setNumThreads(1);
    vgicp->setResolution(1.);
    vgicp->setTransformationEpsilon(0.01);
    vgicp->setMaximumIterations(64);
    vgicp->setCorrespondenceRandomness(20);
    vgicp->setRegularizationMethod(static_cast<fast_gicp::RegularizationMethod>(method));
    return vgicp;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr in, double leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(in);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());
    vg.filter(*out);
    return out;
}

int main(int argc, char** argv)
{
    if (argc != 5)
    {
        std::cerr << "Usage: " << argv[0] << " <pcd-src> <pcd-target> <method> <reg>" << std::endl;
        return 1;
    }
    const auto src_path = argv[1];
    const auto tgt_path = argv[2];
    const auto method = argv[3];
    const int reg = atoi(argv[4]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(src_path, *src_cloud) == -1)
    {
        std::cerr << src_path << " is not a pcd file." << std::endl;
        return 2;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(tgt_path, *tgt_cloud) == -1)
    {
        std::cerr << tgt_path << " is not a pcd file." << std::endl;
        return 2;
    }

    pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration;
    if (strcmp(method, "gicp") == 0)
    {
        registration = FastGICPFactory(reg);
    }
    else if (strcmp(method, "vgicp") == 0)
    {
        registration = FastVGICPFactory(reg);
    }
    else
    {
        std::cerr << method << " is not supported." << std::endl;
        return 2;
    }

    auto src_cloud_filtered = voxel_filter(src_cloud, 0.05);
    auto tgt_cloud_filtered = voxel_filter(tgt_cloud, 0.05);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    registration->setInputSource(src_cloud_filtered);
    registration->setInputTarget(tgt_cloud_filtered);
    registration->align(*aligned_cloud);
    auto transform = registration->getFinalTransformation();
    std::cout << "converged: " << registration->hasConverged() << std::endl;
    std::cout << "transformation: " << std::endl;
    std::cout << registration->getFinalTransformation() << std::endl;

    auto t0 = std::chrono::steady_clock::now();
    auto t1 = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cout << "elapsed: " << elapsed_ms << " [ms]" << std::endl;

    return 0;
}
