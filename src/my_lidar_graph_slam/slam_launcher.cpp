
/* slam_launcher.cpp */

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <queue>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/io/gnuplot_helper.hpp"
#include "my_lidar_graph_slam/io/map_saver.hpp"
#include "my_lidar_graph_slam/io/carmen/carmen_reader.hpp"
#include "my_lidar_graph_slam/mapping/cost_function_greedy_endpoint.hpp"
#include "my_lidar_graph_slam/mapping/lidar_graph_slam.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_greedy_endpoint.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

using namespace MyLidarGraphSlam;

/* Declare namespaces for convenience */
namespace pt = boost::property_tree;

/* Declare types for convenience */
using MapType = GridMap<CountingGridCell<double>>;
using ScanType = Sensor::ScanDataPtr<double>;
using CostType = Mapping::CostGreedyEndpoint<MapType, ScanType>;
using ScanMatcherBaseType = Mapping::ScanMatcher<MapType, ScanType>;
using ScanMatcherType =
    Mapping::ScanMatcherGreedyEndpoint<MapType, ScanType>;

/* Create cost function object */
std::unique_ptr<CostType> CreateCostFunction(
    const pt::ptree& jsonSettings)
{
    /* Load settings for greedy endpoint cost function */
    const double usableRangeMin = jsonSettings.get(
        "CostGreedyEndpoint.UsableRangeMin", 0.01);
    const double usableRangeMax = jsonSettings.get(
        "CostGreedyEndpoint.UsableRangeMax", 50.0);
    const double hitAndMissedDist = jsonSettings.get(
        "CostGreedyEndpoint.HitAndMissedDist", 0.075);
    const double occupancyThreshold = jsonSettings.get(
        "CostGreedyEndpoint.OccupancyThreshold", 0.1);
    const double gaussianSigma = jsonSettings.get(
        "CostGreedyEndpoint.GaussianSigma", 0.05);
    const int kernelSize = jsonSettings.get(
        "CostGreedyEndpoint.KernelSize", 1);
    const double scalingFactor = jsonSettings.get(
        "CostGreedyEndpoint.ScalingFactor", 1.0);
    
    /* Create cost function object */
    std::unique_ptr<CostType> pCostFunc = std::make_unique<CostType>(
        usableRangeMin, usableRangeMax, hitAndMissedDist,
        occupancyThreshold, gaussianSigma, kernelSize,
        scalingFactor);

    return pCostFunc;
}

/* Create scan matcher object */
std::unique_ptr<ScanMatcherType> CreateScanMatcher(
    const pt::ptree& jsonSettings,
    std::unique_ptr<CostType>&& pCostFunc)
{
    /* Load settings for greedy endpoint scan matcher */
    const double linearStep = jsonSettings.get(
        "ScanMatcherGreedyEndpoint.LinearStep", 0.1);
    const double angularStep = jsonSettings.get(
        "ScanMatcherGreedyEndpoint.AngularStep", 0.1);
    const int maxIterations = jsonSettings.get(
        "ScanMatcherGreedyEndpoint.MaxIterations", 100);
    const int maxNumOfRefinements = jsonSettings.get(
        "ScanMatcherGreedyEndpoint.MaxNumOfRefinements", 5);
    
    /* Construct scan matcher */
    std::unique_ptr<ScanMatcherType> pScanMatcher =
        std::make_unique<ScanMatcherType>(
            linearStep, angularStep, maxIterations, maxNumOfRefinements,
            std::move(pCostFunc));
    
    return pScanMatcher;
}

/* Create LiDAR Graph-Based SLAM object */
std::shared_ptr<Mapping::LidarGraphSlam> CreateLidarGraphSlam(
    const pt::ptree& jsonSettings)
{
    /* Create greedy endpoint cost function */
    auto pCostFunc = CreateCostFunction(jsonSettings);
    /* Create greedy endpoint scan matcher */
    auto pScanMatcher = CreateScanMatcher(
        jsonSettings, std::move(pCostFunc));
    
    /* Load settings for LiDAR Graph-Based SLAM */
    const double mapResolution = jsonSettings.get(
        "LidarGraphSlam.Map.Resolution", 0.05);
    const int patchSize = jsonSettings.get(
        "LidarGraphSlam.Map.PatchSize", 64);
    const int numOfScansForLatestMap = jsonSettings.get(
        "LidarGraphSlam.Map.NumOfScansForLatestMap", 5);
    const double travelDistThresholdForLocalMap = jsonSettings.get(
        "LidarGraphSlam.Map.TravelDistThresholdForLocalMap", 10.0);
    
    const double initialPoseX = jsonSettings.get(
        "LidarGraphSlam.InitialPose.X", 0.0);
    const double initialPoseY = jsonSettings.get(
        "LidarGraphSlam.InitialPose.Y", 0.0);
    const double initialPoseTheta = jsonSettings.get(
        "LidarGraphSlam.InitialPose.Theta", 0.0);
    
    const double updateThresholdTravelDist = jsonSettings.get(
        "LidarGraphSlam.UpdateThresholdTravelDist", 1.0);
    const double updateThresholdAngle = jsonSettings.get(
        "LidarGraphSlam.UpdateThresholdAngle", 0.5);
    const double updateThresholdTime = jsonSettings.get(
        "LidarGraphSlam.UpdateThresholdTime", 5.0);

    const double usableRangeMin = jsonSettings.get(
        "LidarGraphSlam.UsableRangeMin", 0.01);
    const double usableRangeMax = jsonSettings.get(
        "LidarGraphSlam.UsableRangeMax", 50.0);
    
    const RobotPose2D<double> initialPose {
        initialPoseX, initialPoseY, initialPoseTheta };
    
    /* Create LiDAR Graph-Based SLAM object */
    std::shared_ptr<Mapping::LidarGraphSlam> pLidarGraphSlam =
        std::make_shared<Mapping::LidarGraphSlam>(
            std::move(pScanMatcher), mapResolution, patchSize,
            numOfScansForLatestMap, travelDistThresholdForLocalMap,
            initialPose, updateThresholdTravelDist, updateThresholdAngle,
            updateThresholdTime, usableRangeMin, usableRangeMax);
    
    return pLidarGraphSlam;
}

int main(int argc, char** argv)
{
    namespace fs = std::filesystem;

    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << ' '
                  << "<Carmen log file name> "
                  << "<JSON Settings file name> "
                  << "[Output name]" << std::endl;
        return EXIT_FAILURE;
    }

    fs::path logFilePath { argv[1] };
    fs::path settingsFilePath { argv[2] };

    /* Determine the output file name */
    const bool hasValidFileName = logFilePath.has_stem() &&
                                  logFilePath.stem() != "." &&
                                  logFilePath.stem() != "..";
    fs::path outputFilePath { (argc == 4 || !hasValidFileName) ? argv[3] :
                              logFilePath.stem() };
    
    /* Open the carmen log file */
    std::ifstream logFile { logFilePath };

    if (!logFile) {
        std::cerr << "Failed to open log file: "
                  << logFilePath.c_str() << std::endl;
        return EXIT_FAILURE;
    }

    /* Load the carmen log file */
    std::vector<Sensor::SensorDataPtr> logData;
    IO::Carmen::CarmenLogReader logReader;
    logReader.Load(logFile, logData);
    logFile.close();

    /* Load settings from JSON file */
    pt::ptree jsonSettings;
    pt::read_json(settingsFilePath, jsonSettings);

    /* Construct LiDAR Graph-Based SLAM */
    auto pLidarGraphSlam = CreateLidarGraphSlam(jsonSettings);

    /* Read the settings for Gnuplot GUI */
    const bool guiEnabled =
        jsonSettings.get<bool>("GuiEnabled", true);
    const int drawFrameInterval =
        jsonSettings.get("DrawFrameInterval", 5);
    
    std::cerr << "Carmen log file loaded: "
              << logFilePath.c_str() << std::endl
              << "JSON setting file loaded: "
              << settingsFilePath.c_str() << std::endl
              << "Output name: "
              << outputFilePath.c_str() << std::endl;

    IO::GnuplotHelper gnuplotHelper;

    for (const auto& sensorData : logData) {
        Sensor::ScanDataPtr<double> scanData =
            std::dynamic_pointer_cast<Sensor::ScanData<double>>(sensorData);
        
        if (scanData == nullptr)
            continue;
        
        /* Process the latest scan data */
        const bool mapUpdated = pLidarGraphSlam->ProcessScan(
            scanData, scanData->OdomPose());
        
        if (!guiEnabled || !mapUpdated)
            continue;
        if (pLidarGraphSlam->ProcessCount() % drawFrameInterval != 0)
            continue;
        
        /* Draw the current pose graph if necessary */
        gnuplotHelper.DrawPoseGraph(pLidarGraphSlam->GetPoseGraph());
    }

    IO::MapSaver mapSaver;
    mapSaver.SaveMap(pLidarGraphSlam->GetGridMapBuilder(),
                     pLidarGraphSlam->GetPoseGraph(),
                     outputFilePath, true);
    mapSaver.SavePoseGraph(pLidarGraphSlam->GetPoseGraph(),
                           outputFilePath);

    return EXIT_SUCCESS;
}
