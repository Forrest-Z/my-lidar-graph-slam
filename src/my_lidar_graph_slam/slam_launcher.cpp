
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
#include "my_lidar_graph_slam/mapping/loop_closure_grid_search.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_optimizer_spchol.hpp"
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
std::shared_ptr<CostType> CreateCostFunction(
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
    std::shared_ptr<CostType> pCostFunc = std::make_shared<CostType>(
        usableRangeMin, usableRangeMax, hitAndMissedDist,
        occupancyThreshold, gaussianSigma, kernelSize,
        scalingFactor);

    return pCostFunc;
}

/* Create scan matcher object */
std::shared_ptr<ScanMatcherType> CreateScanMatcher(
    const pt::ptree& jsonSettings,
    const std::shared_ptr<CostType>& pCostFunc)
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
    std::shared_ptr<ScanMatcherType> pScanMatcher =
        std::make_shared<ScanMatcherType>(
            linearStep, angularStep, maxIterations, maxNumOfRefinements,
            pCostFunc);
    
    return pScanMatcher;
}

/* Create loop closure object */
std::shared_ptr<Mapping::LoopClosureGridSearch> CreateLoopClosure(
    const pt::ptree& jsonSettings,
    const std::shared_ptr<ScanMatcherType>& pScanMatcher,
    const std::shared_ptr<CostType>& pCostFunc)
{
    /* Read settings for loop closure */
    const double nodeDistMax = jsonSettings.get(
        "LoopClosureGridSearch.PoseGraphNodeDistMax", 2.0);
    const double rangeX = jsonSettings.get(
        "LoopClosureGridSearch.SearchRangeX", 2.0);
    const double rangeY = jsonSettings.get(
        "LoopClosureGridSearch.SearchRangeY", 2.0);
    const double rangeTheta = jsonSettings.get(
        "LoopClosureGridSearch.SearchRangeTheta", 1.0);
    const double stepX = jsonSettings.get(
        "LoopClosureGridSearch.SearchStepX", 0.1);
    const double stepY = jsonSettings.get(
        "LoopClosureGridSearch.SearchStepY", 0.1);
    const double stepTheta = jsonSettings.get(
        "LoopClosureGridSearch.SearchStepTheta", 0.05);
    const double costThreshold = jsonSettings.get(
        "LoopClosureGridSearch.CostThreshold", 0.01);

    /* Construct loop closure object */
    std::shared_ptr<Mapping::LoopClosureGridSearch> pLoopClosure =
        std::make_shared<Mapping::LoopClosureGridSearch>(
            pScanMatcher, pCostFunc,
            nodeDistMax, rangeX, rangeY, rangeTheta,
            stepX, stepY, stepTheta, costThreshold);
    
    return pLoopClosure;
}

/* Create pose graph object */
std::shared_ptr<Mapping::PoseGraph> CreatePoseGraph(
    const pt::ptree& jsonSettings)
{
    /* Construct pose graph object */
    std::shared_ptr<Mapping::PoseGraph> pPoseGraph =
        std::make_shared<Mapping::PoseGraph>();
    
    return pPoseGraph;
}

/* Create pose graph optimizer object */
std::shared_ptr<Mapping::PoseGraphOptimizerSpChol> CreatePoseGraphOptimizer(
    const pt::ptree& jsonSettings)
{
    /* Read settings for sparse Cholesky pose graph optimizer */
    const int numOfIterationsMax = jsonSettings.get(
        "PoseGraphOptimizerSpChol.NumOfIterationsMax", 10);
    const double errorTolerance = jsonSettings.get(
        "PoseGraphOptimizerSpChol.ErrorTolerance", 1e-3);
    const double initialLambda = jsonSettings.get(
        "PoseGraphOptimizerSpChol.InitialLambda", 1e-4);

    /* Construct pose graph optimizer object */
    std::shared_ptr<Mapping::PoseGraphOptimizerSpChol> pPoseGraphOptimizer =
        std::make_shared<Mapping::PoseGraphOptimizerSpChol>(
            numOfIterationsMax, errorTolerance, initialLambda);
    
    return pPoseGraphOptimizer;
}

/* Create grid map builder object */
std::shared_ptr<Mapping::GridMapBuilder> CreateGridMapBuilder(
    const pt::ptree& jsonSettings)
{
    /* Load settings for grid map builder */
    const double mapResolution = jsonSettings.get(
        "GridMapBuilder.Map.Resolution", 0.05);
    const int patchSize = jsonSettings.get(
        "GridMapBuilder.Map.PatchSize", 64);
    const int numOfScansForLatestMap = jsonSettings.get(
        "GridMapBuilder.Map.NumOfScansForLatestMap", 5);
    const double travelDistThresholdForLocalMap = jsonSettings.get(
        "GridMapBuilder.Map.TravelDistThresholdForLocalMap", 10.0);
    
    const double usableRangeMin = jsonSettings.get(
        "GridMapBuilder.UsableRangeMin", 0.01);
    const double usableRangeMax = jsonSettings.get(
        "GridMapBuilder.UsableRangeMax", 50.0);
    
    /* Construct grid map builder object */
    std::shared_ptr<Mapping::GridMapBuilder> pGridMapBuilder =
        std::make_shared<Mapping::GridMapBuilder>(
            mapResolution, patchSize,
            numOfScansForLatestMap, travelDistThresholdForLocalMap,
            usableRangeMin, usableRangeMax);
    
    return pGridMapBuilder;
}

/* Create LiDAR Graph-Based SLAM object */
std::shared_ptr<Mapping::LidarGraphSlam> CreateLidarGraphSlam(
    const pt::ptree& jsonSettings)
{
    /* Create grid map builder object */
    auto pGridMapBuilder = CreateGridMapBuilder(jsonSettings);
    /* Create greedy endpoint cost function */
    auto pCostFunc = CreateCostFunction(jsonSettings);
    /* Create greedy endpoint scan matcher */
    auto pScanMatcher = CreateScanMatcher(jsonSettings, pCostFunc);
    /* Create pose graph object */
    auto pPoseGraph = CreatePoseGraph(jsonSettings);
    /* Create pose graph optimizer object */
    auto pPoseGraphOptimizer = CreatePoseGraphOptimizer(jsonSettings);
    /* Create loop closure object */
    auto pLoopClosure = CreateLoopClosure(
        jsonSettings, pScanMatcher, pCostFunc);

    /* Load settings for LiDAR Graph-Based SLAM */
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
    
    const RobotPose2D<double> initialPose {
        initialPoseX, initialPoseY, initialPoseTheta };
    
    /* Create LiDAR Graph-Based SLAM object */
    std::shared_ptr<Mapping::LidarGraphSlam> pLidarGraphSlam =
        std::make_shared<Mapping::LidarGraphSlam>(
            pGridMapBuilder, pScanMatcher,
            pPoseGraph, pPoseGraphOptimizer,
            pLoopClosure, 10, initialPose,
            updateThresholdTravelDist, updateThresholdAngle,
            updateThresholdTime);
    
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
    const bool guiEnabled = jsonSettings.get<bool>("GuiEnabled", true);
    const int drawFrameInterval = jsonSettings.get("DrawFrameInterval", 5);
    
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
    
    /* Wait for key input */
    const bool waitKey = jsonSettings.get<bool>("Launcher.WaitForKey", false);

    if (waitKey)
        std::getchar();

    return EXIT_SUCCESS;
}
