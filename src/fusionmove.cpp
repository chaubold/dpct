#include "fusionmove.h"
#include <memory>

namespace dpct
{

FusionMove::FusionMove(Graph *graph):
    graph_(graph)
{}

std::shared_ptr<Graph> FusionMove::graphUnion(TrackingAlgorithm::Solution& sol_a,
                                  TrackingAlgorithm::Solution& sol_b)
{
    Graph::NodeSelectionMap node_selection_map = graph_->getEmptyNodeSelectionMap();
    Graph::ArcSelectionMap arc_selection_map = graph_->getEmptyArcSelectionMap();

    // select all arcs used in SolutionA
    for(TrackingAlgorithm::Path& p : sol_a)
    {
        for(Arc* a : p)
        {
            graph_->selectArc(node_selection_map, arc_selection_map, a);
        }
    }

    // select all arcs used in SolutionB
    for(TrackingAlgorithm::Path& p : sol_b)
    {
        for(Arc* a : p)
        {
            graph_->selectArc(node_selection_map, arc_selection_map, a);
        }
    }

    return std::make_shared<Graph>(*graph_, node_selection_map, arc_selection_map);
}

//double FusionMove::track(std::vector<Path> &paths)
//{
    // create N copies of the graph
    // #pragma omp parallel for
        // run a perturbed Magnusson on each -> get proposal_paths

    // current_best = proposal_paths[0]
    // for i in range(1,N):
    //     create graph union:
    //     create boolean mask containing all used arcs and nodes of current_best and proposal_paths[i]
    //     copy masked arcs and nodes
    //     contract all arcs that are the only connectors
    //
    //     run tracking on contracted graph union
    //     decode result: construct paths from origin data

    // return current_best
//}

} // namespace dpct
