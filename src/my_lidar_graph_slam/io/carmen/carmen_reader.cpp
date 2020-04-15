
/* carmen_reader.cpp */

#include "my_lidar_graph_slam/io/carmen/carmen_reader.hpp"

namespace MyLidarGraphSlam {
namespace IO {
namespace Carmen {

/* Load Carmen log from input stream */
bool CarmenLogReader::Load(
    std::istream& inputStream,
    std::vector<Sensor::SensorDataPtr>& sensorData)
{
    /* Clear the sensor data vector first */
    sensorData.clear();

    /* Parameters in carmen log file */
    ParamMapType paramMap;

    /* Read Carmen log line by line */
    std::string sensorDataStr;
    std::string sensorId;

    while (std::getline(inputStream, sensorDataStr)) {
        std::istringstream strStream { sensorDataStr };

        /* Skip empty line */
        if (!strStream)
            continue;
        
        /* Read data type (sensor id string) */
        strStream >> sensorId;
        /* Convert to data type enum */
        DataType dataType = this->ToDataType(sensorId);

        /* Read one data record */
        this->ReadLine(sensorId, dataType, strStream, paramMap, sensorData);
    }

    return true;
}

/* Read one data record */
void CarmenLogReader::ReadLine(
    const std::string& sensorId,
    const DataType dataType,
    std::istringstream& strStream,
    ParamMapType& paramMap,
    std::vector<Sensor::SensorDataPtr>& sensorData)
{
    switch (dataType) {
        case DataType::Param: {
            /* Read parameter (For PARAM) */
            this->ReadParameter(strStream, paramMap);
            break;
        }

        case DataType::Odom: {
            /* Read odometry data (For ODOM) */
            auto odomData = this->ReadOdometryData(
                sensorId, strStream, paramMap);
            if (odomData != nullptr)
                sensorData.emplace_back(std::move(odomData));
            break;
        }

        case DataType::RawLaser: {
            /* Read scan data (new format) (For RAWLASER) */
            auto scanData = this->ReadRawLaserData(
                sensorId, strStream, paramMap);
            if (scanData != nullptr)
                sensorData.emplace_back(std::move(scanData));
            break;
        }

        case DataType::RobotLaser: {
            /* Read scan data (new format) (For ROBOTLASER) */
            auto scanData = this->ReadRobotLaserData(
                sensorId, strStream, paramMap);
            if (scanData != nullptr)
                sensorData.emplace_back(std::move(scanData));
            break;
        }

        case DataType::OldFrontLaser:
        case DataType::OldRearLaser: {
            /* Read scan data (old format) (For FLASER, RLASER) */
            auto oldScanData = this->ReadOldLaserData(
                sensorId, strStream, paramMap);
            if (oldScanData != nullptr)
                sensorData.emplace_back(std::move(oldScanData));
            break;
        }

        case DataType::OldOtherLaser: {
            /* Read scan data (old format) (For LASER) */
            auto oldScanData = this->ReadOldOtherLaserData(
                sensorId, strStream, paramMap);
            if (oldScanData != nullptr)
                sensorData.emplace_back(std::move(oldScanData));
            break;
        }

        default:
            /* Ignore other data type */
            break;
    }
}

/* Read parameter */
void CarmenLogReader::ReadParameter(
    std::istringstream& strStream,
    ParamMapType& paramMap)
{
    /* Ignore if parameter name does not exist */
    if (!strStream)
        return;
    
    /* Read the parameter name */
    std::string paramName;
    strStream >> paramName;

    /* Read the parameter value */
    std::string paramValue;
    
    if (strStream)
        strStream >> paramValue;
    
    /* Append to the parameter list */
    paramMap.insert(std::make_pair(paramName, paramValue));
}

/* Read odometry data from log */
Sensor::OdometryDataPtr<double> CarmenLogReader::ReadOdometryData(
    const std::string& sensorId,
    std::istringstream& strStream,
    const ParamMapType& /* paramMap */)
{
    CarmenDataHeader dataHeader;
    RobotPose2D<double> robotPose { 0.0, 0.0, 0.0 };
    RobotPose2D<double> robotVelocity { 0.0, 0.0, 0.0 };
    double robotAcceleration = 0.0;

    /* Error check ignored for simplicity */
    /* Read robot pose */
    strStream >> robotPose.mX >> robotPose.mY >> robotPose.mTheta;
    /* Read robot velocity (might be 0) */
    strStream >> robotVelocity.mX >> robotVelocity.mTheta;
    /* Read robot acceleration (might be 0) */
    strStream >> robotAcceleration;
    /* Read timestamp and host name */
    strStream >> dataHeader.mIpcTimeStamp
              >> dataHeader.mIpcHostName
              >> dataHeader.mLoggerTimeStamp;

    return std::make_shared<Sensor::OdometryData<double>>(
        sensorId, dataHeader.mIpcTimeStamp,
        robotPose, robotVelocity);
}

/* Read scan data from log (new format) */
Sensor::ScanDataPtr<double> CarmenLogReader::ReadRawLaserData(
    const std::string& sensorId,
    std::istringstream& strStream,
    const ParamMapType& /* paramMap */)
{
    CarmenDataHeader dataHeader;

    /* Laser range finder configuration fields */
    int laserType;
    double startAngle;
    double fieldOfView;
    double angularResolution;
    double maxRange;
    double accuracy;
    int remissionMode;

    /* Range value fields */
    int numReadings;
    double rangeValue;
    std::vector<double> rangeValues;

    /* Remission value fields */
    int numRemissions;
    double remissionValue;

    /* Error check ignored for simplicity */
    /* Read laser configuration fields */
    strStream >> laserType >> startAngle
              >> fieldOfView >> angularResolution
              >> maxRange >> accuracy
              >> remissionMode;

    /* Read the number of range values */
    strStream >> numReadings;
    rangeValues.reserve(static_cast<std::size_t>(numReadings));

    /* Read range values */
    for (int i = 0; i < numReadings; ++i) {
        strStream >> rangeValue;
        rangeValues.emplace_back(rangeValue);
    }

    /* Read the number of remission values */
    strStream >> numRemissions;

    /* Read remission values (just ignore) */
    for (int i = 0; i < numRemissions; ++i)
        strStream >> remissionValue;
    
    /* Read timestamp and host name */
    strStream >> dataHeader.mIpcTimeStamp
              >> dataHeader.mIpcHostName
              >> dataHeader.mLoggerTimeStamp;

    const double maxAngle = startAngle +
        angularResolution * static_cast<double>(numReadings - 1);

    return std::make_shared<Sensor::ScanData<double>>(
        sensorId, dataHeader.mIpcTimeStamp,
        RobotPose2D<double>(0.0, 0.0, 0.0),
        RobotPose2D<double>(0.0, 0.0, 0.0),
        RobotPose2D<double>(0.0, 0.0, 0.0),
        0.0, maxRange, startAngle, maxAngle, angularResolution,
        std::move(rangeValues));
}

/* Read scan data from log (new format) */
Sensor::ScanDataPtr<double> CarmenLogReader::ReadRobotLaserData(
    const std::string& sensorId,
    std::istringstream& strStream,
    const ParamMapType& /* paramMap */)
{
    CarmenDataHeader dataHeader;

    /* Laser range finder configuration fields */
    int laserType;
    double startAngle;
    double fieldOfView;
    double angularResolution;
    double maxRange;
    double accuracy;
    int remissionMode;

    /* Range value fields */
    int numReadings;
    double rangeValue;
    std::vector<double> rangeValues;

    /* Pose of robot and laser scanner in world frame */
    RobotPose2D<double> laserPose { 0.0, 0.0, 0.0 };
    RobotPose2D<double> robotPose { 0.0, 0.0, 0.0 };
    RobotPose2D<double> laserVelocity { 0.0, 0.0, 0.0 };
    double forwardSafetyDist;
    double sideSafetyDist;
    double turnAxis;

    /* Error check ignored for simplicity */
    /* Read laser configuration fields */
    strStream >> laserType >> startAngle
              >> fieldOfView >> angularResolution
              >> maxRange >> accuracy
              >> remissionMode;
    
    /* Read the number of range values */
    strStream >> numReadings;
    rangeValues.reserve(static_cast<std::size_t>(numReadings));

    /* Read range values */
    for (int i = 0; i < numReadings; ++i) {
        strStream >> rangeValue;
        rangeValues.emplace_back(rangeValue);
    }

    /* Read the laser pose */
    strStream >> laserPose.mX >> laserPose.mY >> laserPose.mTheta;
    /* Read the robot pose */
    strStream >> robotPose.mX >> robotPose.mY >> robotPose.mTheta;
    /* Read the laser velocity */
    strStream >> laserVelocity.mX >> laserVelocity.mTheta;
    /* Read other parameters */
    strStream >> forwardSafetyDist >> sideSafetyDist >> turnAxis;
    /* Read timestamp and host name */
    strStream >> dataHeader.mIpcTimeStamp
              >> dataHeader.mIpcHostName
              >> dataHeader.mLoggerTimeStamp;
    
    const double maxAngle = startAngle +
        angularResolution * static_cast<double>(numReadings - 1);
    
    return std::make_shared<Sensor::ScanData<double>>(
        sensorId, dataHeader.mIpcTimeStamp,
        robotPose, laserVelocity,
        InverseCompound(robotPose, laserPose),
        0.0, maxRange, startAngle, maxAngle, angularResolution,
        std::move(rangeValues));
}

/* Read scan data from log (old format) */
Sensor::ScanDataPtr<double> CarmenLogReader::ReadOldLaserData(
    const std::string& sensorId,
    std::istringstream& strStream,
    const ParamMapType& paramMap)
{
    CarmenDataHeader dataHeader;

    /* Range value fields */
    int numReadings;
    double rangeValue;
    std::vector<double> rangeValues;

    /* Pose of robot and laser scanner in world frame */
    RobotPose2D<double> laserPose { 0.0, 0.0, 0.0 };
    RobotPose2D<double> robotPose { 0.0, 0.0, 0.0 };
    
    /* Read the number of range values */
    strStream >> numReadings;
    rangeValues.reserve(static_cast<std::size_t>(numReadings));

    /* Read range values */
    for (int i = 0; i < numReadings; ++i) {
        strStream >> rangeValue;
        rangeValues.emplace_back(rangeValue);
    }

    /* Read the laser pose */
    strStream >> laserPose.mX >> laserPose.mY >> laserPose.mTheta;
    /* Read the robot pose */
    strStream >> robotPose.mX >> robotPose.mY >> robotPose.mTheta;
    /* Read timestamp and host name */
    strStream >> dataHeader.mIpcTimeStamp
              >> dataHeader.mIpcHostName
              >> dataHeader.mLoggerTimeStamp;
    
    /* Read parameters for laser scanner */
    const auto paramMinRangeIt = paramMap.find("Laser.MinRange");
    const double minRange = (paramMinRangeIt != paramMap.end()) ?
        std::stod(paramMinRangeIt->second) : 0.0;
    
    const auto paramMaxRangeIt = paramMap.find("Laser.MaxRange");
    const double maxRange = (paramMaxRangeIt != paramMap.end()) ?
        std::stod(paramMaxRangeIt->second) : 80.0;
    
    const auto paramAngleIncrementIt = paramMap.find("Laser.AngleIncrement");
    const double angleIncrement = (paramAngleIncrementIt != paramMap.end()) ?
        std::stod(paramAngleIncrementIt->second) :
        this->GuessAngleIncrement(numReadings);
    
    const auto paramMinAngleIt = paramMap.find("Laser.MinAngle");
    const double minAngle = (paramMinAngleIt != paramMap.end()) ?
        std::stod(paramMinAngleIt->second) : (-M_PI_2);
    
    const auto paramMaxAngleIt = paramMap.find("Laser.MaxAngle");
    const double maxAngle = (paramMaxAngleIt != paramMap.end()) ?
        std::stod(paramMaxAngleIt->second) :
        (paramAngleIncrementIt != paramMap.end()) ?
        minAngle + angleIncrement * static_cast<double>(numReadings) :
        minAngle + this->GuessAngleRange(numReadings);

    return std::make_shared<Sensor::ScanData<double>>(
        sensorId, dataHeader.mIpcTimeStamp,
        robotPose, RobotPose2D<double>(0.0, 0.0, 0.0),
        InverseCompound(robotPose, laserPose),
        minRange, maxRange, minAngle, maxAngle, angleIncrement,
        std::move(rangeValues));
}

/* Read scan data from log (old format) */
Sensor::ScanDataPtr<double> CarmenLogReader::ReadOldOtherLaserData(
    const std::string& sensorId,
    std::istringstream& strStream,
    const ParamMapType& paramMap)
{
    CarmenDataHeader dataHeader;

    /* Range value fields */
    int numReadings;
    double rangeValue;
    std::vector<double> rangeValues;

    /* Read the number of range values */
    strStream >> numReadings;
    rangeValues.reserve(static_cast<std::size_t>(numReadings));

    /* Read range values */
    for (int i = 0; i < numReadings; ++i) {
        strStream >> rangeValue;
        rangeValues.emplace_back(rangeValue);
    }

    /* Read parameters for laser scanner */
    const auto paramMinRangeIt = paramMap.find("Laser.MinRange");
    const double minRange = (paramMinRangeIt != paramMap.end()) ?
        std::stod(paramMinRangeIt->second) : 0.0;
    
    const auto paramMaxRangeIt = paramMap.find("Laser.MaxRange");
    const double maxRange = (paramMaxRangeIt != paramMap.end()) ?
        std::stod(paramMaxRangeIt->second) : 80.0;
    
    const auto paramAngleIncrementIt = paramMap.find("Laser.AngleIncrement");
    const double angleIncrement = (paramAngleIncrementIt != paramMap.end()) ?
        std::stod(paramAngleIncrementIt->second) :
        this->GuessAngleIncrement(numReadings);
    
    const auto paramMinAngleIt = paramMap.find("Laser.MinAngle");
    const double minAngle = (paramMinAngleIt != paramMap.end()) ?
        std::stod(paramMinAngleIt->second) : (-M_PI_2);
    
    const auto paramMaxAngleIt = paramMap.find("Laser.MaxAngle");
    const double maxAngle = (paramMaxAngleIt != paramMap.end()) ?
        std::stod(paramMaxAngleIt->second) :
        (paramAngleIncrementIt != paramMap.end()) ?
        minAngle + angleIncrement * static_cast<double>(numReadings) :
        minAngle + this->GuessAngleRange(numReadings);

    return std::make_shared<Sensor::ScanData<double>>(
        sensorId, dataHeader.mIpcTimeStamp,
        RobotPose2D<double>(0.0, 0.0, 0.0),
        RobotPose2D<double>(0.0, 0.0, 0.0),
        RobotPose2D<double>(0.0, 0.0, 0.0),
        minRange, maxRange, minAngle, maxAngle, angleIncrement,
        std::move(rangeValues));
}

/* Guess the angle increment from the number of scan points */
double CarmenLogReader::GuessAngleRange(int numReadings)
{
    switch (numReadings) {
        case 181:
            return M_PI;
        case 180:
            return M_PI * 179.0 / 180.0;
        case 361:
            return M_PI;
        case 360:
            return M_PI * 179.5 / 180.0;
        case 401:
            return M_PI * 100.0 / 180.0;
        case 400:
            return M_PI * 99.75 / 180.0;
        default:
            return M_PI;
    }
}

/* Guess the angle increment from the number of scan points */
double CarmenLogReader::GuessAngleIncrement(int numReadings)
{
    switch (numReadings) {
        case 181:
            return M_PI / 180.0;
        case 180:
            return M_PI / 180.0;
        case 361:
            return M_PI / 360.0;
        case 360:
            return M_PI / 360.0;
        case 401:
            return M_PI / 720.0;
        case 400:
            return M_PI / 720.0;
        default:
            return CarmenLogReader::GuessAngleRange(numReadings) /
                static_cast<double>(numReadings - 1);
    }
}

/* Convert data type string to enum value */
CarmenLogReader::DataType CarmenLogReader::ToDataType(
    const std::string& dataTypeStr)
{
    using MapType = std::unordered_map<std::string, DataType>;

    static const MapType dataTypeMap {
        { "PARAM",          DataType::Param         },
        { "ODOM",           DataType::Odom          },
        { "TRUEPOS",        DataType::TruePos       },
        { "RAWLASER1",      DataType::RawLaser      },
        { "RAWLASER2",      DataType::RawLaser      },
        { "RAWLASER3",      DataType::RawLaser      },
        { "RAWLASER4",      DataType::RawLaser      },
        { "ROBOTLASER1",    DataType::RobotLaser    },
        { "ROBOTLASER2",    DataType::RobotLaser    },
        { "FLASER",         DataType::OldFrontLaser },
        { "RLASER",         DataType::OldRearLaser  },
        { "LASER3",         DataType::OldOtherLaser },
        { "LASER4",         DataType::OldOtherLaser },
    };

    auto foundIt = dataTypeMap.find(dataTypeStr);
    return (foundIt != dataTypeMap.end()) ?
        foundIt->second : DataType::None;
}

} /* namespace Carmen */
} /* namespace IO */
} /* namespace MyLidarGraphSlam */
