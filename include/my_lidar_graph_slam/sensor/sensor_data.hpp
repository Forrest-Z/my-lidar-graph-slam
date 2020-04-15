
/* sensor_data.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_SENSOR_SENSOR_DATA_HPP
#define MY_LIDAR_GRAPH_SLAM_SENSOR_SENSOR_DATA_HPP

#include <memory>
#include <string>
#include <vector>

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
             const T& angleIncrement,
             std::vector<T>&& ranges) :
        SensorData(sensorId, timeStamp),
        mOdomPose(odomPose),
        mVelocity(velocity),
        mRelativeSensorPose(relativeSensorPose),
        mMinRange(minRange),
        mMaxRange(maxRange),
        mMinAngle(minAngle),
        mMaxAngle(maxAngle),
        mAngleIncrement(angleIncrement),
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
    /* Retrieve the angle between two measurements in radians */
    inline T AngleIncrement() const { return this->mAngleIncrement; }

    /* Retrieve the range data */
    inline const std::vector<T>& Ranges() const { return this->mRanges; }

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
    /* Angle between two measurements in radians */
    T               mAngleIncrement;
    /* Range data */
    std::vector<T>  mRanges;
};

/* Type definitions */
using SensorDataPtr = std::shared_ptr<SensorData>;

template <typename T>
using OdometryDataPtr = std::shared_ptr<OdometryData<T>>;

template <typename T>
using ScanDataPtr = std::shared_ptr<ScanData<T>>;

} /* namespace Sensor */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_SENSOR_SENSOR_DATA_HPP */
