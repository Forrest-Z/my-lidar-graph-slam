
/* scan_interpolator.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_SENSOR_SCAN_INTERPOLATOR_HPP
#define MY_LIDAR_GRAPH_SLAM_SENSOR_SCAN_INTERPOLATOR_HPP

#include <cassert>
#include <memory>
#include <vector>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Type definitions for convenience */
class ScanInterpolator;
using ScanInterpolatorPtr = std::shared_ptr<ScanInterpolator>;

class ScanInterpolator final
{
public:
    /* Constructor */
    ScanInterpolator(const double distScans,
                     const double distThresholdEmpty) :
        mDistScans(distScans),
        mDistThresholdEmpty(distThresholdEmpty) { }

    /* Destructor */
    ~ScanInterpolator() = default;

    /* Interpolate scan data */
    Sensor::ScanDataPtr<double> Interpolate(
        const Sensor::ScanDataPtr<double>& scanData) const;

private:
    /* Distance between two scan points */
    const double mDistScans;
    /* Distance threshold for empty space */
    const double mDistThresholdEmpty;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_SENSOR_SCAN_INTERPOLATOR_HPP */
