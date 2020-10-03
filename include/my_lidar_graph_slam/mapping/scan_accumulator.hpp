
/* scan_accumulator.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_ACCUMULATOR_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_ACCUMULATOR_HPP

#include <deque>
#include <memory>
#include <vector>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Type definitions for convenience */
class ScanAccumulator;
using ScanAccumulatorPtr = std::shared_ptr<ScanAccumulator>;

class ScanAccumulator final
{
public:
    /* Type definitions */
    using ScanDataDeque = std::deque<Sensor::ScanDataPtr<double>>;

    /* Constructor */
    ScanAccumulator(const std::size_t numOfAccumulatedScans);

    /* Destructor */
    ~ScanAccumulator() = default;

    /* Append the scan data */
    void AppendScan(const Sensor::ScanDataPtr<double>& scanData);

    /* Retrieve the concatenated scan data  */
    Sensor::ScanDataPtr<double> ComputeConcatenatedScan();

private:
    /* Number of the scan data to be accumulated */
    const std::size_t   mNumOfAccumulatedScans;
    /* Accumulated scans */
    ScanDataDeque mAccumulatedScans;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_ACCUMULATOR_HPP */
