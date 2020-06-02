
/* scan_accumulator.cpp */

#include "my_lidar_graph_slam/mapping/scan_accumulator.hpp"

#include <cassert>

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
ScanAccumulator::ScanAccumulator(std::size_t numOfAccumulatedScans) :
    mNumOfAccumulatedScans(numOfAccumulatedScans)
{
    assert(numOfAccumulatedScans > 0);
}

/* Append the scan data */
void ScanAccumulator::AppendScan(
    const Sensor::ScanDataPtr<double>& scanData)
{
    this->mAccumulatedScans.push_front(scanData);
}

/* Retrieve the concatenated scan data */
Sensor::ScanDataPtr<double> ScanAccumulator::ComputeConcatenatedScan()
{
    assert(!this->mAccumulatedScans.empty());

    const auto latestScan = this->mAccumulatedScans.front();
    const RobotPose2D<double>& latestScanPose = latestScan->OdomPose();
    const RobotPose2D<double> latestSensorPose =
        Compound(latestScanPose, latestScan->RelativeSensorPose());
    this->mAccumulatedScans.pop_front();

    if (this->mAccumulatedScans.empty())
        return latestScan;

    /* Initialize the ranges and angles with the latest scan */
    std::vector<double> accumulatedRanges = latestScan->Ranges();
    std::vector<double> accumulatedAngles = latestScan->Angles();

    /* Accumulate the previous scan data */
    const std::size_t numOfScans = std::min(
        this->mAccumulatedScans.size(), this->mNumOfAccumulatedScans - 1);

    for (std::size_t scanIdx = 0; scanIdx < numOfScans; ++scanIdx) {
        const auto& scanData = this->mAccumulatedScans[scanIdx];
        const RobotPose2D<double>& scanPose = scanData->OdomPose();
        const RobotPose2D<double> sensorPose =
            Compound(scanPose, scanData->RelativeSensorPose());
        const RobotPose2D<double> relPose =
            InverseCompound(sensorPose, latestSensorPose);

        for (std::size_t i = 0; i < scanData->NumOfScans(); ++i) {
            /* Retrieve the range and angle at the previous frame */
            const double range = scanData->RangeAt(i);
            const double angle = scanData->AngleAt(i);
            const double sinTheta = std::sin(angle);
            const double cosTheta = std::cos(angle);

            /* Compute the range and angle at the latest frame from
             * that of the previous frame using the law of cosines */
            const double newRange = std::sqrt(
                range * range +
                relPose.mX * relPose.mX + relPose.mY * relPose.mY -
                2.0 * range * (relPose.mX * cosTheta + relPose.mY * sinTheta));
            const double scanX = range * cosTheta - relPose.mX;
            const double scanY = range * sinTheta - relPose.mY;
            const double newAngle = NormalizeAngle(
                std::atan2(scanY, scanX) - relPose.mTheta);

            /* Append the range and angle */
            accumulatedRanges.push_back(newRange);
            accumulatedAngles.push_back(newAngle);
        }
    }

    /* Clear the accumulated scans */
    this->mAccumulatedScans.clear();

    /* Create a new scan data */
    auto accumulatedScanData = std::make_shared<Sensor::ScanData<double>>(
        latestScan->SensorId(), latestScan->TimeStamp(),
        latestScan->OdomPose(), latestScan->Velocity(),
        latestScan->RelativeSensorPose(),
        latestScan->MinRange(), latestScan->MaxRange(),
        latestScan->MinAngle(), latestScan->MaxAngle(),
        std::move(accumulatedAngles),
        std::move(accumulatedRanges));

    return accumulatedScanData;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
