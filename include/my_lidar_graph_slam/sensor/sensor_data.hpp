
/* sensor_data.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_SENSOR_SENSOR_DATA_HPP
#define MY_LIDAR_GRAPH_SLAM_SENSOR_SENSOR_DATA_HPP

#include <memory>
#include <string>
#include <vector>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"

namespace MyLidarGraphSlam {
namespace Sensor {

class SensorData
{
public:
    /* Constructor */
    SensorData(const std::string& sensorId, double timeStamp) :
        mSensorId(sensorId), mTimeStamp(timeStamp) { }
    /* Destructor */
    virtual ~SensorData() = default;

    /* Retrieve sensor id string */
    inline const std::string& SensorId() const { return this->mSensorId; }
    /* Retrieve timestamp */
    inline double TimeStamp() const { return this->mTimeStamp; }

protected:
    std::string     mSensorId;
    double          mTimeStamp;
};

template <typename T>
class OdometryData final : public SensorData
{
public:
    /* Constructor */
    OdometryData(const std::string& sensorId,
                 double timeStamp,
                 const RobotPose2D<T>& pose,
                 const RobotPose2D<T>& velocity) :
        SensorData(sensorId, timeStamp),
        mPose(pose),
        mVelocity(velocity) { }

    /* Destructor */
    ~OdometryData() = default;

    /* Retrieve the odometry pose in world frame */
    inline const RobotPose2D<T>& Pose() const { return this->mPose; }
    /* Retrieve the velocity */
    inline const RobotPose2D<T>& Velocity() const { return this->mVelocity; }

private:
    /* Odometry pose in world frame */
    RobotPose2D<T> mPose;
    /* Velocity */
    RobotPose2D<T> mVelocity;
};

template <typename T>
class ScanData final : public SensorData
{
public:
    /* Constructor */
    ScanData(const std::string& sensorId,
             double timeStamp,
             const RobotPose2D<T>& odomPose,
             const RobotPose2D<T>& velocity,
             const RobotPose2D<T>& relativeSensorPose,
             const T& minRange,
             const T& maxRange,
             const T& minAngle,
             const T& maxAngle,
             std::vector<T>&& angles,
             std::vector<T>&& ranges) :
        SensorData(sensorId, timeStamp),
        mOdomPose(odomPose),
        mVelocity(velocity),
        mRelativeSensorPose(relativeSensorPose),
        mMinRange(minRange),
        mMaxRange(maxRange),
        mMinAngle(minAngle),
        mMaxAngle(maxAngle),
        mAngles(angles),
        mRanges(ranges) { }
    
    /* Destructor */
    ~ScanData() = default;

    /* Retrieve the odometry pose in world frame */
    inline const RobotPose2D<T>& OdomPose() const
    { return this->mOdomPose; }
    /* Retrieve the robot velocity */
    inline const RobotPose2D<T>& Velocity() const
    { return this->mVelocity; }

    /* Retrieve the relative sensor pose in robot frame */
    inline const RobotPose2D<T>& RelativeSensorPose() const
    { return this->mRelativeSensorPose; }

    /* Retrieve the minimum range in meters */
    inline T MinRange() const { return this->mMinRange; }
    /* Retrieve the maximum range in meters */
    inline T MaxRange() const { return this->mMaxRange; }
    /* Retrieve the minimum angle in radians */
    inline T MinAngle() const { return this->mMinAngle; }
    /* Retrieve the maximum angle in radians */
    inline T MaxAngle() const { return this->mMaxAngle; }

    /* Retrieve the angle data */
    inline const std::vector<T>& Angles() const { return this->mAngles; }
    /* Retrieve the range data */
    inline const std::vector<T>& Ranges() const { return this->mRanges; }
    /* Retrieve the number of scans */
    inline std::size_t NumOfScans() const { return this->mRanges.size(); }

    /* Retrieve the scan range at the specified index */
    inline T RangeAt(std::size_t scanIdx) const
    { return this->mRanges.at(scanIdx); }
    /* Retrieve the scan angle at the specified index */
    inline T AngleAt(std::size_t scanIdx) const
    { return this->mAngles.at(scanIdx); }

    /* Compute the global hit point of the specified scan */
    Point2D<double> GlobalHitPoint(
        const RobotPose2D<double>& globalSensorPose,
        const std::size_t scanIdx) const;

    /* Compute the global hit pose of the specified scan */
    RobotPose2D<double> GlobalHitPose(
        const RobotPose2D<double>& globalSensorPose,
        const std::size_t scanIdx) const;

    /* Compute the local hit point of the specified scan */
    Point2D<double> LocalHitPoint(
        const std::size_t scanIdx) const;

    /* Compute the local hit pose of the specified scan */
    RobotPose2D<double> LocalHitPose(
        const std::size_t scanIdx) const;

    /* Compute the hit point and missed point of the specified scan */
    void HitAndMissedPoint(const RobotPose2D<double>& sensorPose,
                           std::size_t scanIdx,
                           T hitAndMissedDist,
                           Point2D<double>& hitPoint,
                           Point2D<double>& missedPoint) const;

private:
    /* Odometry pose in world frame (required) */
    RobotPose2D<T>  mOdomPose;
    /* Robot velocity (optional) */
    RobotPose2D<T>  mVelocity;
    /* Relative sensor pose in robot frame */
    RobotPose2D<T>  mRelativeSensorPose;
    /* Minimum range in meters */
    T               mMinRange;
    /* Maximum range in meters */
    T               mMaxRange;
    /* Minimum angle in radians */
    T               mMinAngle;
    /* Maximum angle in radians */
    T               mMaxAngle;
    /* Angle data */
    std::vector<T>  mAngles;
    /* Range data */
    std::vector<T>  mRanges;
};

/* Compute the global hit point of the specified scan */
template <typename T>
Point2D<double> ScanData<T>::GlobalHitPoint(
    const RobotPose2D<double>& globalSensorPose,
    const std::size_t scanIdx) const
{
    const T scanRange = this->RangeAt(scanIdx);
    const T scanAngle = this->AngleAt(scanIdx);
    const T cosTheta = std::cos(globalSensorPose.mTheta + scanAngle);
    const T sinTheta = std::sin(globalSensorPose.mTheta + scanAngle);

    return Point2D<double> {
        globalSensorPose.mX + scanRange * cosTheta,
        globalSensorPose.mY + scanRange * sinTheta };
}

/* Compute the global hit pose of the specified scan */
template <typename T>
RobotPose2D<double> ScanData<T>::GlobalHitPose(
    const RobotPose2D<double>& globalSensorPose,
    const std::size_t scanIdx) const
{
    const T scanRange = this->RangeAt(scanIdx);
    const T scanAngle = this->AngleAt(scanIdx);
    const T cosTheta = std::cos(globalSensorPose.mTheta + scanAngle);
    const T sinTheta = std::sin(globalSensorPose.mTheta + scanAngle);

    return RobotPose2D<double> {
        globalSensorPose.mX + scanRange * cosTheta,
        globalSensorPose.mY + scanRange * sinTheta,
        globalSensorPose.mTheta + scanAngle };
}

/* Compute the local hit point of the specified scan */
template <typename T>
Point2D<double> ScanData<T>::LocalHitPoint(
    const std::size_t scanIdx) const
{
    const T scanRange = this->RangeAt(scanIdx);
    const T scanAngle = this->AngleAt(scanIdx);
    const T cosTheta = std::cos(scanAngle);
    const T sinTheta = std::sin(scanAngle);

    return Point2D<double> {
        scanRange * cosTheta, scanRange * sinTheta };
}

/* Compute the local hit pose of the specified scan */
template <typename T>
RobotPose2D<double> ScanData<T>::LocalHitPose(
    const std::size_t scanIdx) const
{
    const T scanRange = this->RangeAt(scanIdx);
    const T scanAngle = this->AngleAt(scanIdx);
    const T cosTheta = std::cos(scanAngle);
    const T sinTheta = std::sin(scanAngle);

    return RobotPose2D<double> {
        scanRange * cosTheta, scanAngle * sinTheta, scanAngle };
}

/* Compute the hit point and missed point of the specified scan */
template <typename T>
void ScanData<T>::HitAndMissedPoint(
    const RobotPose2D<double>& sensorPose,
    std::size_t scanIdx,
    T hitAndMissedDist,
    Point2D<double>& hitPoint,
    Point2D<double>& missedPoint) const
{
    const T scanRange = this->RangeAt(scanIdx);
    const T scanAngle = this->AngleAt(scanIdx);
    const T cosTheta = std::cos(sensorPose.mTheta + scanAngle);
    const T sinTheta = std::sin(sensorPose.mTheta + scanAngle);

    /* Compute the hit point */
    hitPoint.mX = sensorPose.mX + scanRange * cosTheta;
    hitPoint.mY = sensorPose.mY + scanRange * sinTheta;

    /* Compute the missed point */
    missedPoint.mX = sensorPose.mX + (scanRange - hitAndMissedDist) * cosTheta;
    missedPoint.mY = sensorPose.mY + (scanRange - hitAndMissedDist) * sinTheta;

    return;
}

/* Type definitions */
using SensorDataPtr = std::shared_ptr<const SensorData>;

template <typename T>
using OdometryDataPtr = std::shared_ptr<const OdometryData<T>>;

template <typename T>
using ScanDataPtr = std::shared_ptr<const ScanData<T>>;

} /* namespace Sensor */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_SENSOR_SENSOR_DATA_HPP */
