
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
#include "my_lidar_graph_slam/grid_map/binary_bayes_grid_cell.hpp"
#include "my_lidar_graph_slam/io/gnuplot_helper.hpp"
#include "my_lidar_graph_slam/io/map_saver.hpp"
#include "my_lidar_graph_slam/io/carmen/carmen_reader.hpp"
#include "my_lidar_graph_slam/mapping/cost_function_greedy_endpoint.hpp"
#include "my_lidar_graph_slam/mapping/cost_function_square_error.hpp"
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
using MapType = GridMap<BinaryBayesGridCell<double>>;
using ScanType = Sensor::ScanDataPtr<double>;

using CostFuncType =
    Mapping::CostFunction<MapType, ScanType>;
using CostGreedyEndpointType =
    Mapping::CostGreedyEndpoint<MapType, ScanType>;
using CostSquareErrorType =
    Mapping::CostSquareError<MapType, ScanType>;

using ScanMatcherType =
    Mapping::ScanMatcher<MapType, ScanType>;
using ScanMatcherGreedyEndpointType =
    Mapping::ScanMatcherGreedyEndpoint<MapType, ScanType>;

using LoopClosureType = Mapping::LoopClosure;
using LoopClosureGridSearchType = Mapping::LoopClosureGridSearch;

using OptimizerType = Mapping::PoseGraphOptimizer;
using OptimizerSpCholType = Mapping::PoseGraphOptimizerSpChol;

/* Create the greedy endpoint cost function object */
std::shared_ptr<CostFuncType> CreateCostGreedyEndpoint(
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
    auto pCostFunc = std::make_shared<CostGreedyEndpointType>(
        usableRangeMin, usableRangeMax, hitAndMissedDist,
        occupancyThreshold, gaussianSigma, kernelSize,
        scalingFactor);

    return pCostFunc;
}

/* Create the square error cost function object */
std::shared_ptr<CostFuncType> CreateCostSquareError(
    const pt::ptree& jsonSettings)
{
    /* Load settings for square error cost function */
    const double usableRangeMin = jsonSettings.get(
        "CostSquareError.UsableRangeMin", 0.01);
    const double usableRangeMax = jsonSettings.get(
        "CostSquareError.UsableRangeMax", 50.0);
    
    /* Create cost function object */
    auto pCostFunc = std::make_shared<CostSquareErrorType>(
        usableRangeMin, usableRangeMax);
    
    return pCostFunc;
}

/* Create the cost function object */
std::shared_ptr<CostFuncType> CreateCostFunction(
    const pt::ptree& jsonSettings,
    const std::string& costType)
{
    if (costType == "GreedyEndpoint")
        return CreateCostGreedyEndpoint(jsonSettings);
    else if (costType == "SquareError")
        return CreateCostSquareError(jsonSettings);
    
    return nullptr;
}

/* Create the greedy endpoint scan matcher object */
std::shared_ptr<ScanMatcherType> CreateScanMatcherGreedyEndpoint(
    const pt::ptree& jsonSettings,
    const std::shared_ptr<CostFuncType>& pCostFunc)
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
    auto pScanMatcher = std::make_shared<ScanMatcherGreedyEndpointType>(
        linearStep, angularStep, maxIterations, maxNumOfRefinements,
        pCostFunc);
    
    return pScanMatcher;
}

/* Create the scan matcher object */
std::shared_ptr<ScanMatcherType> CreateScanMatcher(
    const pt::ptree& jsonSettings,
    const std::string& scanMatcherType,
    const std::shared_ptr<CostFuncType>& pCostFunc)
{
    if (scanMatcherType == "GreedyEndpoint")
        return CreateScanMatcherGreedyEndpoint(jsonSettings, pCostFunc);
    
    return nullptr;
}

/* Create the grid search loop closure object */
std::shared_ptr<LoopClosureType> CreateLoopClosureGridSearch(
    const pt::ptree& jsonSettings,
    const std::shared_ptr<ScanMatcherType>& pScanMatcher,
    const std::shared_ptr<CostFuncType>& pCostFunc)
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
    auto pLoopClosure = std::make_shared<Mapping::LoopClosureGridSearch>(
        pScanMatcher, pCostFunc, nodeDistMax,
        rangeX, rangeY, rangeTheta, stepX, stepY, stepTheta, costThreshold);
    
    return pLoopClosure;
}

/* Create the loop closure object */
std::shared_ptr<LoopClosureType> CreateLoopClosure(
    const pt::ptree& jsonSettings,
    const std::string& loopClosureType,
    const std::shared_ptr<ScanMatcherType>& pScanMatcher,
    const std::shared_ptr<CostFuncType>& pCostFunc)
{
    if (loopClosureType == "GridSearch")
        return CreateLoopClosureGridSearch(
            jsonSettings, pScanMatcher, pCostFunc);
    
    return nullptr;
}

/* Create pose graph object */
std::shared_ptr<Mapping::PoseGraph> CreatePoseGraph(
    const pt::ptree& jsonSettings)
{
    /* Construct pose graph object */
    auto pPoseGraph = std::make_shared<Mapping::PoseGraph>();

    return pPoseGraph;
}

/* Create sparse Cholesky pose graph optimizer object */
std::shared_ptr<OptimizerType> CreatePoseGraphOptimizerSpChol(
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
    auto pOptimizer = std::make_shared<OptimizerSpCholType>(
        numOfIterationsMax, errorTolerance, initialLambda);
    
    return pOptimizer;
}

/* Create pose graph optimizer object */
std::shared_ptr<OptimizerType> CreatePoseGraphOptimizer(
    const pt::ptree& jsonSettings,
    const std::string& optimizerType)
{
    if (optimizerType == "SpChol")
        return CreatePoseGraphOptimizerSpChol(jsonSettings);
    
    return nullptr;
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
    
    const double probHit = jsonSettings.get(
        "GridMapBuilder.ProbabilityHit", 0.9);
    const double probMiss = jsonSettings.get(
        "GridMapBuilder.ProbabilityMiss", 0.1);
    
    /* Construct grid map builder object */
    auto pGridMapBuilder = std::make_shared<Mapping::GridMapBuilder>(
        mapResolution, patchSize,
        numOfScansForLatestMap, travelDistThresholdForLocalMap,
        usableRangeMin, usableRangeMax,
        probHit, probMiss);
    
    return pGridMapBuilder;
}

/* Create LiDAR Graph-Based SLAM object */
std::shared_ptr<Mapping::LidarGraphSlam> CreateLidarGraphSlam(
    const pt::ptree& jsonSettings)
{
    /* Create grid map builder object */
    auto pGridMapBuilder = CreateGridMapBuilder(jsonSettings);

    /* Create the cost function for local SLAM */
    const std::string localCostType = jsonSettings.get(
        "LidarGraphSlam.LocalSlam.CostType", "GreedyEndpoint");
    auto pCostFunc = CreateCostFunction(jsonSettings, localCostType);

    /* Create the scan matcher for local SLAM */
    const std::string localScanMatcherType = jsonSettings.get(
        "LidarGraphSlam.LocalSlam.ScanMatcherType", "GreedyEndpoint");
    auto pScanMatcher = CreateScanMatcher(
        jsonSettings, localScanMatcherType, pCostFunc);
    
    /* Create pose graph */
    auto pPoseGraph = CreatePoseGraph(jsonSettings);

    /* Create pose graph optimizer */
    const std::string optimizerType = jsonSettings.get(
        "LidarGraphSlam.PoseGraphOptimizerType", "SpChol");
    auto pOptimizer = CreatePoseGraphOptimizer(jsonSettings, optimizerType);

    /* Create loop closure object */
    const std::string loopClosureType = jsonSettings.get(
        "LidarGraphSlam.LoopClosureType", "GridSearch");
    auto pLoopClosure = CreateLoopClosure(
        jsonSettings, loopClosureType, pScanMatcher, pCostFunc);

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
    
    const int loopClosureInterval = jsonSettings.get(
        "LidarGraphSlam.LoopClosureInterval", 10);
    
    const RobotPose2D<double> initialPose {
        initialPoseX, initialPoseY, initialPoseTheta };

    /* Create LiDAR Graph-Based SLAM object */
    auto pLidarGraphSlam = std::make_shared<Mapping::LidarGraphSlam>(
        pGridMapBuilder, pScanMatcher,
        pPoseGraph, pOptimizer, pLoopClosure,
        loopClosureInterval, initialPose,
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
    mapSaver.SaveLatestMap(pLidarGraphSlam->GetGridMapBuilder(),
                           pLidarGraphSlam->GetPoseGraph(),
                           outputFilePath);
    
    /* Wait for key input */
    const bool waitKey = jsonSettings.get<bool>("Launcher.WaitForKey", false);

    if (waitKey)
        std::getchar();

    return EXIT_SUCCESS;
}
