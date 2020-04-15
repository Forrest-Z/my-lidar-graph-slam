
/* carmen_reader.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_IO_CARMEN_CARMEN_READER_HPP
#define MY_LIDAR_GRAPH_SLAM_IO_CARMEN_CARMEN_READER_HPP

#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace IO {
namespace Carmen {

struct CarmenDataHeader
{
    /* Default constructor */
    CarmenDataHeader() :
        mIpcHostName(), mIpcTimeStamp(0.0), mLoggerTimeStamp(0.0) { }
    
    std::string mIpcHostName;
    double      mIpcTimeStamp;
    double      mLoggerTimeStamp;
};

class CarmenLogReader final
{
private:
    /* Enumerator for Carmen log data type */
    enum class DataType
    {
        None = 0,
        Param,
        Odom,
        TruePos,
        RawLaser,
        RobotLaser,
        OldFrontLaser,
        OldRearLaser,
        OldOtherLaser,
    };

    /* Declare types for convenience */
    using ParamMapType = std::unordered_map<std::string, std::string>;

public:
    /* Constructor */
    CarmenLogReader() = default;
    /* Destructor */
    ~CarmenLogReader() = default;

    /* Load Carmen log from input stream */
    bool Load(
        std::istream& inputStream,
        std::vector<Sensor::SensorDataPtr>& sensorData);

private:
    /* Read one data record */
    void ReadLine(const std::string& sensorId,
                  const DataType dataType,
                  std::istringstream& strStream,
                  ParamMapType& paramMap,
                  std::vector<Sensor::SensorDataPtr>& sensorData);

    /* Read parameter */
    void ReadParameter(std::istringstream& strStream,
                       ParamMapType& paramMap);

    /* Read odometry data from log */
    Sensor::OdometryDataPtr<double> ReadOdometryData(
        const std::string& sensorId,
        std::istringstream& strStream,
        const ParamMapType& paramMap);

    /* Read scan data from log (new format) */
    Sensor::ScanDataPtr<double> ReadRawLaserData(
        const std::string& sensorId,
        std::istringstream& strStream,
        const ParamMapType& paramMap);
    Sensor::ScanDataPtr<double> ReadRobotLaserData(
        const std::string& sensorId,
        std::istringstream& strStream,
        const ParamMapType& paramMap);
    
    /* Read scan data from log (old format) */
    Sensor::ScanDataPtr<double> ReadOldLaserData(
        const std::string& sensorId,
        std::istringstream& strStream,
        const ParamMapType& paramMap);
    Sensor::ScanDataPtr<double> ReadOldOtherLaserData(
        const std::string& sensorId,
        std::istringstream& strStream,
        const ParamMapType& paramMap);

    /* Guess the angle range from the number of scan points */
    static double GuessAngleRange(int numReadings);
    /* Guess the angle increment from the number of scan points */
    static double GuessAngleIncrement(int numReadings);
    
    /* Convert data type string to enum value */
    static DataType ToDataType(const std::string& dataTypeStr);
};

} /* namespace Carmen */
} /* namespace IO */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_IO_CARMEN_CARMEN_READER_HPP */
