#ifndef SCAN_PROJECTOR_H_
#define SCAN_PROJECTOR_H_

#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ptx/types.h>

namespace ptx
{

class ScanProjector
{
public:
    PointsT operator() (const sensor_msgs::LaserScan scan)
    {
        if (angle_min_ != scan.angle_min || angle_increment_ != scan.angle_increment || cos_sin_table_.size() != scan.ranges.size())
        {
            buildTable(scan.angle_min, scan.angle_increment, scan.ranges.size());
        }
        PointsT points;
        points.resize(scan.ranges.size());
        for (size_t i = 0; i < scan.ranges.size(); ++i)
        {
            if (!std::isfinite(scan.ranges[i])) continue;
            points[i].x = cos_sin_table_[i].first * scan.ranges[i];
            points[i].y = cos_sin_table_[i].second * scan.ranges[i];
            points[i].z = 0.f;
            points[i].intensity = 1.;
            if (!scan.intensities.empty() && std::isfinite(scan.intensities[i]))
            {
                points[i].intensity = scan.intensities[i];
            }
        }
        points.header = pcl_conversions::toPCL(scan.header);
        return points;
    }
private:
    double angle_min_;
    double angle_increment_;
    std::vector<std::pair<float, float>> cos_sin_table_;

    void buildTable(double angle_min, double angle_increment, size_t npoints)
    {
        angle_min_ = angle_min;
        angle_increment_ = angle_increment;
        cos_sin_table_.resize(npoints);
        for (size_t i = 0; i < npoints; ++i)
        {
            float angle = angle_min + angle_increment * i;
            cos_sin_table_[i] = {std::cos(angle), std::sin(angle)};
        }
    }
};

}

#endif  //SCAN_PROJECTOR_H_
