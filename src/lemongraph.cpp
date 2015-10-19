#include "lemongraph.h"

#include <assert.h>

#include "graph.h"
#include "node.h"
#include "arc.h"
#include "log.h"


namespace dpct
{

LemonGraph::LemonGraph(Graph* g):
	lemonGraph_(),
	arcScoreMap_(lemonGraph_),
	graph_(g)
{
	assert(g != nullptr);

	auto addLemonNode = [&](Node* n)
	{
		LemonNode ln = lemonGraph_.addNode();
		nodeToLemonNodeMap_[n] = ln;
	};

	// add source and sink
	addLemonNode(&g->sourceNode_);
	addLemonNode(&g->sinkNode_);

	// add all others
	for(const auto& timestep : g->nodesPerTimestep_)
    {
        for(const Graph::NodePtr& np : timestep)
        {
        	addLemonNode(np.get());
        }
    }

    // add all arcs
    for(const Graph::ArcPtr& a : g->arcs_)
    {
    	LemonArc la = lemonGraph_.addArc(nodeToLemonNodeMap_[a->getSourceNode()],
    									 nodeToLemonNodeMap_[a->getTargetNode()]);
    	arcToLemonArcMap_[a.get()] = la;
    }

    LOG_MSG("Created lemon graph with " << lemon::countNodes(lemonGraph_) 
    	     << " nodes and " << lemon::countArcs(lemonGraph_) << " arcs");
}

void LemonGraph::updateArcScores()
{
	double score = 0.0;
    for(const Graph::ArcPtr& a : graph_->arcs_)
    {
    	// TODO: handle disabled arcs!
    	score = a->getScoreDelta() + a->getTargetNode()->getScoreDeltaForCurrentCellCount();
    	arcScoreMap_.set(arcToLemonArcMap_[a.get()], score);
    }
}

} // end namespace dpct
