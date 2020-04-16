
/* gnuplot_helper.cpp */

#include "my_lidar_graph_slam/io/gnuplot_helper.hpp"

namespace MyLidarGraphSlam {
namespace IO {

/* Default Constructor */
GnuplotHelper::GnuplotHelper() :
    mGnuplot(nullptr)
{
    /* Interprocess communication between Gnuplot */
    this->mGnuplot.reset(popen("gnuplot 2>/dev/null", "w"));

    std::fprintf(this->mGnuplot.get(), "set terminal qt\n");
    std::fprintf(this->mGnuplot.get(), "unset key\n");
    std::fprintf(this->mGnuplot.get(), "set size ratio -1\n");
}

/* Draw the pose graph */
void GnuplotHelper::DrawPoseGraph(
    const std::shared_ptr<Mapping::PoseGraph>& poseGraph) const
{
    /* Setup pose graph edges */
    std::fprintf(this->mGnuplot.get(), "$edges << EOF\n");

    for (const auto& edge : poseGraph->Edges()) {
        const auto& startNode = poseGraph->NodeAt(edge.StartNodeIndex());
        const auto& endNode = poseGraph->NodeAt(edge.EndNodeIndex());

        std::fprintf(this->mGnuplot.get(), "%f %f\n%f %f\n\n",
                     startNode.Pose().mX, startNode.Pose().mY,
                     endNode.Pose().mX, endNode.Pose().mY);
    }

    std::fprintf(this->mGnuplot.get(), "EOF\n");

    /* Draw pose graph edges */
    std::fprintf(
        this->mGnuplot.get(),
        "plot $edges using 1:2 with lines lc rgb \"black\" lw 2, \\\n"
        "$edges using 1:2:(0.15) with circles fill solid lc rgb \"red\"\n");
    std::fflush(this->mGnuplot.get());
}

} /* namespace IO */
} /* namespace MyLidarGraphSlam */