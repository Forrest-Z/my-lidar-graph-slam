
/* scan_interpolator.cpp */

#include "my_lidar_graph_slam/mapping/scan_interpolator.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Interpolate scan data */
Sensor::ScanDataPtr<double> ScanInterpolator::Interpolate(
    const Sensor::ScanDataPtr<double>& scanData) const
{
    /* Interpolate scan data so that the distance between two scan points
     * are equalized */
    const std::vector<double> scanRanges = scanData->Ranges();
    const std::vector<double> scanAngles = scanData->Angles();
    assert(scanData->NumOfScans() > 0);
    assert(scanData->NumOfScans() == scanRanges.size());
    assert(scanRanges.size() == scanAngles.size());

    /* Convert polar coordinate to cartesian coordinate */
    std::vector<Point2D<double>> scanPoints;
    scanPoints.reserve(scanData->NumOfScans());

    for (std::size_t i = 0; i < scanData->NumOfScans(); ++i) {
        const Point2D<double> scanPoint =
            ToCartesianCoordinate(scanRanges.at(i), scanAngles.at(i));
        scanPoints.push_back(scanPoint);
    }

    /* Insert the first scan point */
    std::vector<Point2D<double>> interpolatedScanPoints;
    interpolatedScanPoints.push_back(scanPoints.at(0));

    Point2D<double> prevPoint = scanPoints.at(0);
    double accumDist = 0.0;

    /* Interpolate scan data */
    for (std::size_t i = 1; i < scanRanges.size(); ++i) {
        const Point2D<double> point = scanPoints.at(i);
        const double dist = Distance(prevPoint, point);

        if (accumDist + dist < this->mDistScans) {
            /* Do not interpolate the scan point
             * adjacent scan points are too close */
            accumDist += dist;
            prevPoint = point;
        } else if (accumDist + dist >= this->mDistThresholdEmpty) {
            /* The space between two adjacent scan points are considered empty
             * thus do not perform interpolation */
            interpolatedScanPoints.push_back(point);
            prevPoint = point;
            accumDist = 0.0;
        } else {
            /* Interpolate scan points */
            const double ratio = (this->mDistScans - accumDist) / dist;
            const double scanPointX =
                (point.mX - prevPoint.mX) * ratio + prevPoint.mX;
            const double scanPointY =
                (point.mY - prevPoint.mY) * ratio + prevPoint.mY;
            const Point2D<double> scanPoint { scanPointX, scanPointY };
            interpolatedScanPoints.push_back(scanPoint);
            prevPoint = scanPoint;
            accumDist = 0.0;
            /* Process the current scan point again */
            --i;
        }
    }

    /* Convert cartesian coordinate to polar coordinate */
    std::vector<double> interpolatedScanRanges;
    std::vector<double> interpolatedScanAngles;
    interpolatedScanRanges.reserve(interpolatedScanPoints.size());
    interpolatedScanAngles.reserve(interpolatedScanPoints.size());

    for (std::size_t i = 0; i < interpolatedScanPoints.size(); ++i) {
        double scanRange;
        double scanAngle;
        std::tie(scanRange, scanAngle) =
            ToPolarCoordinate(interpolatedScanPoints.at(i));
        interpolatedScanRanges.push_back(scanRange);
        interpolatedScanAngles.push_back(scanAngle);
    }

    const double minAngle = interpolatedScanAngles.front();
    const double maxAngle = interpolatedScanAngles.back();

    /* Create a new scan data */
    auto interpolatedScanData = std::make_shared<Sensor::ScanData<double>>(
        scanData->SensorId(), scanData->TimeStamp(),
        scanData->OdomPose(), scanData->Velocity(),
        scanData->RelativeSensorPose(),
        scanData->MinRange(), scanData->MaxRange(),
        minAngle, maxAngle,
        std::move(interpolatedScanAngles),
        std::move(interpolatedScanRanges));

    return interpolatedScanData;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
