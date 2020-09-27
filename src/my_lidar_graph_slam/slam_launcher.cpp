
/* slam_launcher.cpp */

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <vector>

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
#include "my_lidar_graph_slam/mapping/lidar_graph_slam_backend.hpp"
#include "my_lidar_graph_slam/mapping/lidar_graph_slam_frontend.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_branch_bound.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_empty.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_grid_search.hpp"
#include "my_lidar_graph_slam/mapping/loop_searcher.hpp"
#include "my_lidar_graph_slam/mapping/loop_searcher_nearest.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_optimizer_spchol.hpp"
#include "my_lidar_graph_slam/mapping/scan_interpolator.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_hill_climbing.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_linear_solver.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_real_time_correlative.hpp"
#include "my_lidar_graph_slam/mapping/score_function.hpp"
#include "my_lidar_graph_slam/mapping/score_function_pixel_accurate.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

using namespace MyLidarGraphSlam;

/* Declare namespaces for convenience */
namespace pt = boost::property_tree;
namespace fs = std::filesystem;

/* Create the greedy endpoint cost function object */
std::shared_ptr<Mapping::CostFunction> CreateCostGreedyEndpoint(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for greedy endpoint cost function */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const double usableRangeMin = config.get("UsableRangeMin", 0.01);
    const double usableRangeMax = config.get("UsableRangeMax", 50.0);
    const double hitAndMissedDist = config.get("HitAndMissedDist", 0.075);
    const double occupancyThreshold = config.get("OccupancyThreshold", 0.1);
    const int kernelSize = config.get("KernelSize", 1);
    const double scalingFactor = config.get("ScalingFactor", 1.0);

    /* Create greedy endpoint cost function */
    auto pCostFunc = std::make_shared<Mapping::CostGreedyEndpoint>(
        usableRangeMin, usableRangeMax, hitAndMissedDist,
        occupancyThreshold, kernelSize, scalingFactor);

    return pCostFunc;
}

/* Create the square error cost function object */
std::shared_ptr<Mapping::CostFunction> CreateCostSquareError(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for square error cost function */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const double usableRangeMin = config.get("UsableRangeMin", 0.01);
    const double usableRangeMax = config.get("UsableRangeMax", 50.0);
    
    /* Create cost function object */
    auto pCostFunc = std::make_shared<Mapping::CostSquareError>(
        usableRangeMin, usableRangeMax);
    
    return pCostFunc;
}

/* Create the cost function object */
std::shared_ptr<Mapping::CostFunction> CreateCostFunction(
    const pt::ptree& jsonSettings,
    const std::string& costType,
    const std::string& configGroup)
{
    if (costType == "GreedyEndpoint")
        return CreateCostGreedyEndpoint(jsonSettings, configGroup);
    else if (costType == "SquareError")
        return CreateCostSquareError(jsonSettings, configGroup);
    
    return nullptr;
}

/* Create the pixel-accurate score function object */
std::shared_ptr<Mapping::ScoreFunction> CreateScorePixelAccurate(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for pixel-accurate score function */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const double usableRangeMin = config.get("UsableRangeMin", 0.01);
    const double usableRangeMax = config.get("UsableRangeMax", 50.0);

    /* Create score function object */
    auto pScoreFunc = std::make_shared<Mapping::ScorePixelAccurate>(
        usableRangeMin, usableRangeMax);
    
    return pScoreFunc;
}

/* Create the score function object */
std::shared_ptr<Mapping::ScoreFunction> CreateScoreFunction(
    const pt::ptree& jsonSettings,
    const std::string& scoreType,
    const std::string& configGroup)
{
    if (scoreType == "PixelAccurate")
        return CreateScorePixelAccurate(jsonSettings, configGroup);
    
    return nullptr;
}

/* Create the greedy endpoint scan matcher object */
std::shared_ptr<Mapping::ScanMatcher> CreateScanMatcherHillClimbing(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for hill-climbing based scan matcher */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const double linearStep = config.get("LinearStep", 0.1);
    const double angularStep = config.get("AngularStep", 0.1);
    const int maxIterations = config.get("MaxIterations", 100);
    const int maxNumOfRefinements = config.get("MaxNumOfRefinements", 5);

    /* Construct cost function */
    const std::string costType =
        config.get("CostType", "GreedyEndpoint");
    const std::string costConfigGroup =
        config.get("CostConfigGroup", "CostGreedyEndpoint");
    auto pCostFunc = CreateCostFunction(
        jsonSettings, costType, costConfigGroup);

    /* Construct scan matcher */
    auto pScanMatcher = std::make_shared<Mapping::ScanMatcherHillClimbing>(
        linearStep, angularStep, maxIterations, maxNumOfRefinements,
        pCostFunc);
    
    return pScanMatcher;
}

/* Create the linear solver scan matcher object */
std::shared_ptr<Mapping::ScanMatcher> CreateScanMatcherLinearSolver(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for linear solver scan matcher */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const int numOfIterationsMax = config.get("NumOfIterationsMax", 3);
    const double errorTolerance = config.get("ConvergenceThreshold", 1e-2);
    const double usableRangeMin = config.get("UsableRangeMin", 0.01);
    const double usableRangeMax = config.get("UsableRangeMax", 50.0);
    const double transWeight = config.get("TranslationRegularizer", 1e-3);
    const double rotWeight = config.get("RotationRegularizer", 1e-3);

    /* Construct square error cost function */
    const std::string costType =
        config.get("CostType", "SquareError");
    const std::string costConfigGroup =
        config.get("CostConfigGroup", "CostSquareError");
    
    /* Check if the cost is calculated based on the squared error */
    assert(costType == "SquareError");

    auto pCostFunc = std::dynamic_pointer_cast<Mapping::CostSquareError>(
        CreateCostSquareError(jsonSettings, costConfigGroup));

    /* Construct scan matcher */
    auto pScanMatcher = std::make_shared<Mapping::ScanMatcherLinearSolver>(
        numOfIterationsMax, errorTolerance, usableRangeMin, usableRangeMax,
        transWeight, rotWeight, pCostFunc);
    
    return pScanMatcher;
}

/* Create the real-time correlative scan matcher object */
std::shared_ptr<Mapping::ScanMatcher> CreateScanMatcherRealTimeCorrelative(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for real-time correlative scan matcher */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const int lowResolution = config.get("LowResolutionMapWinSize", 10);
    const double rangeX = config.get("SearchRangeX", 0.75);
    const double rangeY = config.get("SearchRangeY", 0.75);
    const double rangeTheta = config.get("SearchRangeTheta", 0.5);
    const double scanRangeMax = config.get("ScanRangeMax", 20.0);

    /* Construct cost function */
    const std::string costType =
        config.get("CostType", "GreedyEndpoint");
    const std::string costConfigGroup =
        config.get("CostConfigGroup", "CostGreedyEndpoint");
    auto pCostFunc = CreateCostFunction(
        jsonSettings, costType, costConfigGroup);

    /* Construct the real-time correlative scan matcher */
    auto pScanMatcher = std::make_shared<
        Mapping::ScanMatcherRealTimeCorrelative>(
            pCostFunc, lowResolution, rangeX, rangeY, rangeTheta, scanRangeMax);

    return pScanMatcher;
}

/* Create the scan matcher object */
std::shared_ptr<Mapping::ScanMatcher> CreateScanMatcher(
    const pt::ptree& jsonSettings,
    const std::string& scanMatcherType,
    const std::string& configGroup)
{
    if (scanMatcherType == "HillClimbing")
        return CreateScanMatcherHillClimbing(jsonSettings, configGroup);
    else if (scanMatcherType == "LinearSolver")
        return CreateScanMatcherLinearSolver(jsonSettings, configGroup);
    else if (scanMatcherType == "RealTimeCorrelative")
        return CreateScanMatcherRealTimeCorrelative(jsonSettings, configGroup);
    
    return nullptr;
}

/* Create the nearest loop searcher */
std::shared_ptr<Mapping::LoopSearcherNearest> CreateLoopSearcherNearest(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for nearest loop searcher */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const double travelDistThreshold = config.get("TravelDistThreshold", 10.0);
    const double nodeDistMax = config.get("PoseGraphNodeDistMax", 2.0);

    /* Construct loop searcher */
    auto pLoopSearcher = std::make_shared<Mapping::LoopSearcherNearest>(
        travelDistThreshold, nodeDistMax);

    return pLoopSearcher;
}

/* Create the loop searcher */
std::shared_ptr<Mapping::LoopSearcher> CreateLoopSearcher(
    const pt::ptree& jsonSettings,
    const std::string& searcherType,
    const std::string& configGroup)
{
    if (searcherType == "Nearest")
        return CreateLoopSearcherNearest(jsonSettings, configGroup);

    return nullptr;
}

/* Create an empty loop detector object */
std::shared_ptr<Mapping::LoopDetector> CreateLoopDetectorEmpty(
    const pt::ptree& /* jsonSettings */,
    const std::string& /* configGroup */)
{
    /* Construct an empty loop detector object */
    auto pLoopDetector = std::make_shared<Mapping::LoopDetectorEmpty>();
    return pLoopDetector;
}

/* Create a grid search loop detector object */
std::shared_ptr<Mapping::LoopDetector> CreateLoopDetectorGridSearch(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for a grid search loop detector */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const double rangeX = config.get("SearchRangeX", 2.0);
    const double rangeY = config.get("SearchRangeY", 2.0);
    const double rangeTheta = config.get("SearchRangeTheta", 1.0);
    const double stepX = config.get("SearchStepX", 0.1);
    const double stepY = config.get("SearchStepY", 0.1);
    const double stepTheta = config.get("SearchStepTheta", 0.05);
    const double scoreThreshold = config.get("ScoreThreshold", 0.8);
    const double matchRateThreshold = config.get("MatchRateThreshold", 0.8);

    /* Construct score function */
    const std::string scoreType =
        config.get("ScoreType", "PixelAccurate");
    const std::string scoreConfigGroup =
        config.get("ScoreConfigGroup", "ScorePixelAccurate");
    auto pScoreFunc = CreateScoreFunction(
        jsonSettings, scoreType, scoreConfigGroup);

    /* Construct cost function */
    const std::string costType =
        config.get("CostType", "GreedyEndpoint");
    const std::string costConfigGroup =
        config.get("CostConfigGroup", "CostGreedyEndpoint");
    auto pCostFunc = CreateCostFunction(
        jsonSettings, costType, costConfigGroup);

    /* Construct a grid search loop detector object */
    auto pLoopDetector = std::make_shared<Mapping::LoopDetectorGridSearch>(
        pScoreFunc, pCostFunc,
        rangeX, rangeY, rangeTheta, stepX, stepY, stepTheta,
        scoreThreshold, matchRateThreshold);

    return pLoopDetector;
}

/* Create a branch-and-bound loop detector object */
std::shared_ptr<Mapping::LoopDetector> CreateLoopDetectorBranchBound(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for a branch-and-bound loop detector */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const int nodeHeightMax = config.get("NodeHeightMax", 6);
    const double rangeX = config.get("SearchRangeX", 2.0);
    const double rangeY = config.get("SearchRangeY", 2.0);
    const double rangeTheta = config.get("SearchRangeTheta", 1.0);
    const double scanRangeMax = config.get("ScanRangeMax", 20.0);
    const double scoreThreshold = config.get("ScoreThreshold", 0.8);
    const double matchRateThreshold = config.get("MatchRateThreshold", 0.8);

    /* Construct score function */
    const std::string scoreType =
        config.get("ScoreType", "PixelAccurate");
    const std::string scoreConfigGroup =
        config.get("ScoreConfigGroup", "ScorePixelAccurate");
    auto pScoreFunc = CreateScoreFunction(
        jsonSettings, scoreType, scoreConfigGroup);

    /* Construct cost function */
    const std::string costType =
        config.get("CostType", "GreedyEndpoint");
    const std::string costConfigGroup =
        config.get("CostConfigGroup", "CostGreedyEndpoint");
    auto pCostFunc = CreateCostFunction(
        jsonSettings, costType, costConfigGroup);

    /* Construct a branch-and-bound loop detector object */
    auto pLoopDetector = std::make_shared<Mapping::LoopDetectorBranchBound>(
        pScoreFunc, pCostFunc,
        nodeHeightMax, rangeX, rangeY, rangeTheta, scanRangeMax,
        scoreThreshold, matchRateThreshold);

    return pLoopDetector;
}

/* Create a loop detector object */
std::shared_ptr<Mapping::LoopDetector> CreateLoopDetector(
    const pt::ptree& jsonSettings,
    const std::string& loopDetectorType,
    const std::string& configGroup)
{
    if (loopDetectorType == "GridSearch")
        return CreateLoopDetectorGridSearch(jsonSettings, configGroup);
    else if (loopDetectorType == "BranchBound")
        return CreateLoopDetectorBranchBound(jsonSettings, configGroup);
    else if (loopDetectorType == "Empty")
        return CreateLoopDetectorEmpty(jsonSettings, configGroup);

    return nullptr;
}

/* Create pose graph object */
std::shared_ptr<Mapping::PoseGraph> CreatePoseGraph(
    const pt::ptree& /* jsonSettings */)
{
    /* Construct pose graph object */
    auto pPoseGraph = std::make_shared<Mapping::PoseGraph>();
    return pPoseGraph;
}

/* Create sparse Cholesky pose graph optimizer object */
std::shared_ptr<Mapping::PoseGraphOptimizer> CreatePoseGraphOptimizerSpChol(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for sparse Cholesky pose graph optimizer */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const int numOfIterationsMax = config.get("NumOfIterationsMax", 10);
    const double errorTolerance = config.get("ErrorTolerance", 1e-3);
    const double initialLambda = config.get("InitialLambda", 1e-4);

    /* Construct pose graph optimizer object */
    auto pOptimizer = std::make_shared<Mapping::PoseGraphOptimizerSpChol>(
        numOfIterationsMax, errorTolerance, initialLambda);
    
    return pOptimizer;
}

/* Create pose graph optimizer object */
std::shared_ptr<Mapping::PoseGraphOptimizer> CreatePoseGraphOptimizer(
    const pt::ptree& jsonSettings,
    const std::string& optimizerType,
    const std::string& configGroup)
{
    if (optimizerType == "SpChol")
        return CreatePoseGraphOptimizerSpChol(jsonSettings, configGroup);
    
    return nullptr;
}

/* Create scan interpolator object */
std::shared_ptr<Mapping::ScanInterpolator> CreateScanInterpolator(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Read settings for scan interpolator */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const double distScans = config.get("DistScans", 0.05);
    const double distThresholdEmpty = config.get("DistThresholdEmpty", 0.25);

    /* Construct scan interpolator object */
    auto pScanInterpolator = std::make_shared<Mapping::ScanInterpolator>(
        distScans, distThresholdEmpty);

    return pScanInterpolator;
}

/* Create grid map builder object */
std::shared_ptr<Mapping::GridMapBuilder> CreateGridMapBuilder(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for grid map builder */
    const pt::ptree& config = jsonSettings.get_child(configGroup);

    const double mapResolution = config.get("Map.Resolution", 0.05);
    const int patchSize = config.get("Map.PatchSize", 64);
    const int numOfLatestScans =
        config.get("Map.NumOfScansForLatestMap", 5);
    const double localMapTravelDist =
        config.get("Map.TravelDistThresholdForLocalMap", 20.0);
    
    const double usableRangeMin = config.get("UsableRangeMin", 0.01);
    const double usableRangeMax = config.get("UsableRangeMax", 50.0);
    
    const double probHit = config.get("ProbabilityHit", 0.9);
    const double probMiss = config.get("ProbabilityMiss", 0.1);
    
    /* Construct grid map builder object */
    auto pGridMapBuilder = std::make_shared<Mapping::GridMapBuilder>(
        mapResolution, patchSize, numOfLatestScans, localMapTravelDist,
        usableRangeMin, usableRangeMax, probHit, probMiss);
    
    return pGridMapBuilder;
}

/* Create SLAM frontend */
std::shared_ptr<Mapping::LidarGraphSlamFrontend> CreateSlamFrontend(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for SLAM frontend */
    const auto& config = jsonSettings.get_child(configGroup);

    /* Create scan interpolator if necessary */
    const bool useInterpolator =
        config.get("UseScanInterpolator", true);
    const std::string interpolatorConfigGroup =
        config.get("ScanInterpolatorConfigGroup", "ScanInterpolator");

    auto pScanInterpolator = useInterpolator ?
        CreateScanInterpolator(jsonSettings, interpolatorConfigGroup) :
        nullptr;

    /* Create the scan matcher for local SLAM */
    const std::string localScanMatcherType =
        config.get("LocalSlam.ScanMatcherType", "HillClimbing");
    const std::string localScanMatcherConfigGroup =
        config.get("LocalSlam.ScanMatcherConfigGroup",
                   "ScanMatcherHillClimbing");

    auto pScanMatcher = CreateScanMatcher(
        jsonSettings, localScanMatcherType, localScanMatcherConfigGroup);

    /* Load the initial pose */
    const double initialPoseX = config.get("InitialPose.X", 0.0);
    const double initialPoseY = config.get("InitialPose.Y", 0.0);
    const double initialPoseTheta = config.get("InitialPose.Theta", 0.0);

    const RobotPose2D<double> initialPose {
        initialPoseX, initialPoseY, initialPoseTheta };

    /* Load the threshold values for local SLAM */
    const double updateThresholdTravelDist =
        config.get("UpdateThresholdTravelDist", 1.0);
    const double updateThresholdAngle =
        config.get("UpdateThresholdAngle", 0.5);
    const double updateThresholdTime =
        config.get("UpdateThresholdTime", 5.0);

    /* Load settings for trigering loop detection */
    const int loopDetectionInterval = config.get("LoopDetectionInterval", 10);

    /* Create SLAM frontend */
    auto pFrontend = std::make_shared<Mapping::LidarGraphSlamFrontend>(
        pScanInterpolator, pScanMatcher, initialPose,
        updateThresholdTravelDist, updateThresholdAngle, updateThresholdTime,
        loopDetectionInterval);

    return pFrontend;
}

/* Create SLAM backend */
std::shared_ptr<Mapping::LidarGraphSlamBackend> CreateSlamBackend(
    const pt::ptree& jsonSettings,
    const std::string& configGroup)
{
    /* Load settings for SLAM backend */
    const auto& config = jsonSettings.get_child(configGroup);

    /* Create pose graph optimizer */
    const std::string optimizerType =
        config.get("PoseGraphOptimizerType", "SpChol");
    const std::string optimizerConfigGroup =
        config.get("PoseGraphOptimizerConfigGroup",
                   "PoseGraphOptimizerSpChol");
    auto pOptimizer = CreatePoseGraphOptimizer(
        jsonSettings, optimizerType, optimizerConfigGroup);

    /* Create loop searcher */
    const std::string loopSearcherType =
        config.get("LoopSearcherType", "Nearest");
    const std::string loopSearcherConfigGroup =
        config.get("LoopSearcherConfigGroup", "LoopSearcherNearest");
    auto pLoopSearcher = CreateLoopSearcher(
        jsonSettings, loopSearcherType, loopSearcherConfigGroup);

    /* Create loop detector object */
    const std::string loopDetectorType =
        config.get("LoopDetectorType", "GridSearch");
    const std::string loopDetectorConfigGroup =
        config.get("LoopDetectorConfigGroup", "LoopDetectorGridSearch");
    auto pLoopDetector = CreateLoopDetector(
        jsonSettings, loopDetectorType, loopDetectorConfigGroup);

    /* Create SLAM backend */
    auto pBackend = std::make_shared<Mapping::LidarGraphSlamBackend>(
        pOptimizer, pLoopSearcher, pLoopDetector);

    return pBackend;
}

/* Create LiDAR Graph-Based SLAM object */
std::shared_ptr<Mapping::LidarGraphSlam> CreateLidarGraphSlam(
    const pt::ptree& jsonSettings)
{
    /* Load settings for LiDAR Graph-Based SLAM */
    const pt::ptree& config = jsonSettings.get_child("LidarGraphSlam");

    /* Create grid map builder */
    const std::string gridMapBuilderConfigGroup =
        config.get("GridMapBuilderConfigGroup", "GridMapBuilder");
    auto pGridMapBuilder = CreateGridMapBuilder(
        jsonSettings, gridMapBuilderConfigGroup);

    /* Create pose graph */
    auto pPoseGraph = CreatePoseGraph(jsonSettings);

    /* Create SLAM frontend object */
    const std::string frontendConfigGroup =
        config.get("FrontendConfigGroup", "Frontend");
    auto pFrontend = CreateSlamFrontend(jsonSettings, frontendConfigGroup);

    /* Create SLAM backend object */
    const std::string backendConfigGroup =
        config.get("BackendConfigGroup", "Backend");
    auto pBackend = CreateSlamBackend(jsonSettings, backendConfigGroup);

    /* Create LiDAR Graph-Based SLAM object */
    auto pLidarGraphSlam = std::make_shared<Mapping::LidarGraphSlam>(
        pFrontend, pBackend, pGridMapBuilder, pPoseGraph);

    return pLidarGraphSlam;
}

/* Load Carmen log data */
void LoadCarmenLog(const fs::path& logFilePath,
                   std::vector<Sensor::SensorDataPtr>& logData)
{
    /* Open the carmen log file */
    std::ifstream logFile { logFilePath };

    if (!logFile) {
        std::cerr << "Failed to open log file: " << logFilePath << std::endl;
        return;
    }

    /* Load the carmen log file */
    IO::Carmen::CarmenLogReader logReader;
    logReader.Load(logFile, logData);
    logFile.close();
}

int main(int argc, char** argv)
{
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

    /* Load Carmen log file */
    std::vector<Sensor::SensorDataPtr> logData;
    LoadCarmenLog(logFilePath, logData);

    if (logData.empty())
        return EXIT_FAILURE;

    /* Load settings from JSON file */
    pt::ptree jsonSettings;
    pt::read_json(settingsFilePath, jsonSettings);

    /* Construct LiDAR Graph-Based SLAM */
    auto pLidarGraphSlam = CreateLidarGraphSlam(jsonSettings);

    /* Read the settings for Gnuplot GUI */
    const bool guiEnabled =
        jsonSettings.get("Launcher.GuiEnabled", true);
    const int drawFrameInterval =
        jsonSettings.get("Launcher.DrawFrameInterval", 5);
    
    std::cerr << "Carmen log file loaded: "
              << logFilePath.c_str() << std::endl
              << "JSON setting file loaded: "
              << settingsFilePath.c_str() << std::endl
              << "Output name: "
              << outputFilePath.c_str() << std::endl;

    /* Start the SLAM backend */
    pLidarGraphSlam->StartBackend();

    IO::GnuplotHelper gnuplotHelper;

    for (const auto& sensorData : logData) {
        auto scanData = std::dynamic_pointer_cast<
            const Sensor::ScanData<double>>(sensorData);

        if (scanData == nullptr)
            continue;

        /* Process the latest scan data */
        const bool mapUpdated = pLidarGraphSlam->ProcessScan(
            scanData, scanData->OdomPose());

        if (!guiEnabled || !mapUpdated)
            continue;
        if (pLidarGraphSlam->ProcessCount() % drawFrameInterval != 0)
            continue;

        /* Get the pose graph information */
        std::map<int, Mapping::NodePosition> poseGraphNodes;
        std::vector<Mapping::EdgeConnection> poseGraphEdges;
        pLidarGraphSlam->GetPoseGraph(poseGraphNodes, poseGraphEdges);

        /* Draw the current pose graph if necessary */
        gnuplotHelper.DrawPoseGraph(poseGraphNodes, poseGraphEdges);
    }

    /* Stop the SLAM backend */
    pLidarGraphSlam->StopBackend();

    IO::MapSaver* const pMapSaver = IO::MapSaver::Instance();

    /* Retrieve a latest map that contains latest scans */
    int latestMapNodeIdxMin;
    int latestMapNodeIdxMax;
    Mapping::GridMapType latestMap = pLidarGraphSlam->GetLatestMap(
        latestMapNodeIdxMin, latestMapNodeIdxMax);

    /* Build a global map that contains all local grid maps */
    int globalMapNodeIdxMin;
    int globalMapNodeIdxMax;
    Mapping::GridMapType globalMap = pLidarGraphSlam->GetGlobalMap(
        globalMapNodeIdxMin, globalMapNodeIdxMax);

    /* Retrieve all pose graph nodes and edges */
    std::vector<Mapping::PoseGraph::Node> poseGraphNodes;
    std::vector<Mapping::PoseGraph::Edge> poseGraphEdges;
    pLidarGraphSlam->GetPoseGraph(poseGraphNodes, poseGraphEdges);

    /* Save the global map, the pose graph, and the latest map */
    pMapSaver->SaveMap(globalMap, poseGraphNodes,
                       outputFilePath, true, true);
    pMapSaver->SavePoseGraph(poseGraphNodes, poseGraphEdges, outputFilePath);
    pMapSaver->SaveLatestMap(latestMap, poseGraphNodes, true,
                             latestMapNodeIdxMin, latestMapNodeIdxMax,
                             true, outputFilePath);

    /* Wait for key input */
    const bool waitKey = jsonSettings.get("Launcher.WaitForKey", false);

    if (waitKey)
        std::getchar();

    return EXIT_SUCCESS;
}
