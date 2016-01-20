#include "flowgraph.h"
#include "log.h"

#include <assert.h>
#include <limits>

namespace dpct
{
FlowGraph::FlowGraph():
	flowMap_(baseGraph_),
	capacityMap_(baseGraph_)
{
	source_ = baseGraph_.addNode();
	target_ = baseGraph_.addNode();
}

FlowGraph::Node FlowGraph::addNode(const CostVector& costs)
{
	assert(costs.size() > 0);
	
	Node n = baseGraph_.addNode();
	nodeCosts_[n] = costs;

	return n;
}

FlowGraph::Arc FlowGraph::addArc(FlowGraph::Node source,
	FlowGraph::Node target,
	const CostVector& costs)
{
	assert(costs.size() > 0);
	Arc a = baseGraph_.addArc(source, target);
	arcCosts_[a] = costs;
	flowMap_[a] = 0;
	capacityMap_[a] = costs.size();
	return a;
}

/// create duplicated parent node for the given node with the given cost
FlowGraph::Arc FlowGraph::allowMitosis(FlowGraph::Node parent,
	double divisionCost)
{
	// set up duplicate with disabled in arc
	Node duplicate = addNode(nodeCosts_[parent]);
	Arc a = addArc(source_, duplicate, {divisionCost});

	// copy all out arcs
	for(Graph::OutArcIt oa(baseGraph_, parent); oa != lemon::INVALID; ++oa)
	{
		addArc(duplicate, baseGraph_.target(oa), arcCosts_[oa]);
	}

	parentToDuplicateMap_[parent] = duplicate;
	duplicateToParentMap_[duplicate] = parent;

	return a;
}

/// start the tracking
void FlowGraph::maxFlowMinCostTracking()
{
	TimePoint startTime_ = std::chrono::high_resolution_clock::now();

	initializeResidualGraph();

	ResidualGraph::ShortestPathResult result;
	size_t iter=0;
	do
	{
		TimePoint iterationStartTime = std::chrono::high_resolution_clock::now();
		DEBUG_MSG("\t>>> Iteration");
		DEBUG_MSG("Current Flow:");
		printAllFlows();

		result = residualGraph_->findShortestPath(source_, target_);

		DEBUG_MSG("\tFound path or cycle"
				<< " of length " << result.first.size() 
				<< " of distance " << result.second);

		if(result.first.size() > 0 && result.second < 0.0)
		{
#ifdef DEBUG_LOG
			printPath(result.first); std::cout << std::endl;
			// std::stringstream outName;
			// outName << "/Users/chaubold/Desktop/residualGraph_iter" << iter << ".dot";
			// residualGraph_->toDot(outName.str(), result.first);
#endif
			augmentUnitFlow(result.first);
			updateEnabledArcs(result.first);
		}
		TimePoint iterationEndTime = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed_seconds = iterationEndTime - iterationStartTime;
		DEBUG_MSG("\t<<<Iteration " << iter++ << " done in " << elapsed_seconds.count() << " secs");
	}
	while(result.first.size() > 0 && result.second < 0.0);

	cleanUpDuplicatedOutArcs();

	TimePoint endTime_ = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed_seconds = endTime_ - startTime_;
	LOG_MSG("Tracking took " << elapsed_seconds.count() << " secs and " << iter << " iterations");
}

void FlowGraph::cleanUpDuplicatedOutArcs()
{
	for(std::map<Node, Node>::iterator it = duplicateToParentMap_.begin(); it != duplicateToParentMap_.end(); ++it)
	{
		for(Graph::OutArcIt oa(baseGraph_, it->first); oa != lemon::INVALID; ++oa)
		{
			if(flowMap_[oa] != 0)
				flowMap_[lemon::findArc(baseGraph_, it->second, baseGraph_.target(oa))] = flowMap_[oa];
		}
	}
}

/// use the current flow and arcEnabled maps to create a residual graph using the appropriate cost deltas
void FlowGraph::initializeResidualGraph()
{
	residualGraph_ = std::make_shared<ResidualGraph>(baseGraph_);
	
	for(Graph::ArcIt a(baseGraph_); a != lemon::INVALID; ++a)
    {
    	updateArc(a);
    	if(duplicateToParentMap_.find(baseGraph_.target(a)) != duplicateToParentMap_.end())
    		enableArc(a, false);
    }
}

void FlowGraph::printPath(const Path& p)
{
	for(auto a : p)
    {
        std::cout << "(" << baseGraph_.id(baseGraph_.target(a.first)) << ", " 
        		<< baseGraph_.id(baseGraph_.source(a.first)) << "):" << a.second << " ";
    }
}

/// augment flow along a path or cycle, adding one unit of flow forward, and subtracting one backwards
void FlowGraph::augmentUnitFlow(const FlowGraph::Path& p)
{
	for(const std::pair<Arc, int>& af : p)
	{
		flowMap_[af.first] += af.second;
		updateArc(af.first);
	}
}

void FlowGraph::enableArc(const Arc& a, bool state)
{
	residualGraph_->enableArc(a, state);
}

double FlowGraph::getArcCost(const Arc& a, int flow)
{
	if(flow >= 0 && flow < arcCosts_[a].size())
	{
		return arcCosts_[a][flow];
	}
	else
		return std::numeric_limits<double>::infinity();
}

double FlowGraph::getNodeCost(const Node& n, int flow)
{
	if(n == source_ || n == target_)
		return 0.0;
	else
	{
		if(flow >= 0 && flow < nodeCosts_[n].size())
			return nodeCosts_[n][flow];
		else
			return std::numeric_limits<double>::infinity();
	}
}

void FlowGraph::updateArc(const Arc& a)
{
	int flow = flowMap_[a];
	int capacity = capacityMap_[a];

	// forward arc:
	double forwardCost = getArcCost(a, flow) + getNodeCost(baseGraph_.target(a), flow);
    residualGraph_->updateArc(a, ResidualGraph::Forward, forwardCost, capacity - flow);

    // backward arc:
    double backwardCost = -1.0 * (getArcCost(a, flow - 1) + getNodeCost(baseGraph_.target(a), flow - 1));
    residualGraph_->updateArc(a, ResidualGraph::Backward, backwardCost, flow);
}

/// updates the arcEnabled map by checking which divisions should be enabled/disabled after this track
void FlowGraph::updateEnabledArcs(const FlowGraph::Path& p)
{
	// define functions for enabling / disabling
	auto toggleOutArcs = [&](const Node& n, bool state)
	{
		DEBUG_MSG("Setting out arcs of " << (baseGraph_.id(n)) << " to " << (state?"true":"false"));
		for(Graph::OutArcIt oa(baseGraph_, n); oa != lemon::INVALID; ++oa)
			enableArc(oa, state);
	};

	auto toggleInArcs = [&](const Node& n, bool state)
	{
		DEBUG_MSG("Setting in arcs of " << baseGraph_.id(n) << " to " << (state?"true":"false"));
		for(Graph::InArcIt ia(baseGraph_, n); ia != lemon::INVALID; ++ia)
			enableArc(ia, state);
	};

	auto toggleOutArcsBut = [&](const Node& n, const Node& exception, bool state)
	{
		DEBUG_MSG("Setting out arcs of " << baseGraph_.id(n) 
			<< " but " << baseGraph_.id(exception) 
			<< " to " << (state?"true":"false"));
		for(Graph::OutArcIt oa(baseGraph_, n); oa != lemon::INVALID; ++oa)
		{
			if(baseGraph_.target(oa) != exception)
				enableArc(oa, state);
		}
	};

	auto toggleInArcsBut = [&](const Node& n, const Node& exception, bool state)
	{
		DEBUG_MSG("Setting in arcs of " << baseGraph_.id(n) 
			<< " but " << baseGraph_.id(exception)
			<< " to " << (state?"true":"false"));
		for(Graph::InArcIt ia(baseGraph_, n); ia != lemon::INVALID; ++ia)
		{
			if(baseGraph_.source(ia) != exception)
				enableArc(ia, state);
		}
	};

	auto toggleDivision = [&](const Node& div, const Node& target, bool divState)
	{
		DEBUG_MSG("Setting division " << baseGraph_.id(div) << " to " << (divState?"true":"false"));
		for(Graph::InArcIt ia(baseGraph_, div); ia != lemon::INVALID; ++ia)
		{
			DEBUG_MSG("\ttoggling division arc " << baseGraph_.id(baseGraph_.source(ia)) << ", " << baseGraph_.id(baseGraph_.target(ia)));
			enableArc(ia, divState);
		}

		for(Graph::OutArcIt oa(baseGraph_, div); oa != lemon::INVALID; ++oa)
		{
			if(baseGraph_.target(oa) == target)
			{
				DEBUG_MSG("\ttoggling move arc " << baseGraph_.id(baseGraph_.source(oa)) << ", " << baseGraph_.id(baseGraph_.target(oa)));
				enableArc(oa, !divState);
				return;
			}
		}
		DEBUG_MSG("was not able to find the arc to " << baseGraph_.id(target) << "!");
	};

	auto toggleAppearanceArc = [&](const Node& n, bool state)
	{
		for(Graph::InArcIt ia(baseGraph_, n); ia != lemon::INVALID; ++ia)
		{
			if(baseGraph_.source(ia) == source_)
			{
				DEBUG_MSG("Setting appearance of " << baseGraph_.id(n) << " to " << (state?"true":"false"));
				enableArc(ia, state);
				return;
			}
		}
		DEBUG_MSG("Didn't find appearance arc of " << baseGraph_.id(n));
	};

	auto toggleDisappearanceArc = [&](const Node& n, bool state)
	{
		for(Graph::OutArcIt oa(baseGraph_, n); oa != lemon::INVALID; ++oa)
		{
			if(baseGraph_.target(oa) == target_)
			{
				DEBUG_MSG("Setting disappearance of " << baseGraph_.id(n) << " to " << (state?"true":"false"));
				enableArc(oa, state);
				return;
			}
		}
		DEBUG_MSG("Didn't find disappearance arc of " << baseGraph_.id(n));
	};

	// check all arcs on path whether they toggle other arc states
	for(const std::pair<Arc, int>& af : p)
	{
		bool forward = af.second > 0;
		Node source = baseGraph_.source(af.first);
		Node target = baseGraph_.target(af.first);

		DEBUG_MSG("Updating stuff for " << (forward? "forward" : "backward") << " edge from " 
			<< baseGraph_.id(source) << " to " << baseGraph_.id(target));

		// division updates
		if(parentToDuplicateMap_.find(source) != parentToDuplicateMap_.end())
		{
			if(sumOutFlow(source) == 1)
			{
				// we have exactly one unit of flow forward through a parent node -> allows division
				toggleDivision(parentToDuplicateMap_[source], target, true);
			}
			else
			{
				// in all other cases, a division is not allowed
				toggleDivision(parentToDuplicateMap_[source], target, false);
			}
		}
		else if(duplicateToParentMap_.find(source) != duplicateToParentMap_.end())
		{
			if(forward)
			{
				// adding flow through division -> parent cannot be undone
				toggleOutArcs(duplicateToParentMap_[source], false);
			}
			else
			{
				// removing flow from division -> parent can be undone again
				toggleOutArcs(duplicateToParentMap_[source], true);
			}
		}

		// partial appearance/disappearance
		if(source == source_)
		{
			// changing usage of appearance arc, enable/disable all other incomings to target
			toggleInArcsBut(target, source, sumInFlow(target) == 0);
		}
		else if(target == target_)
		{
			// changing usage of disappearance arc, enable/disable all other outgoings of source
			toggleOutArcsBut(source, target, sumOutFlow(source) == 0);
		}
		else
		{
			// we did not use an appearance or disappearance arc! 
			// enable those if no other in-/out- flow at that arc yet
			toggleDisappearanceArc(source, sumOutFlow(source) == 0);
			toggleAppearanceArc(target, sumInFlow(target) == 0);
		}
	}
}

void FlowGraph::printAllFlows()
{
	for(Graph::ArcIt a(baseGraph_); a != lemon::INVALID; ++a)
	{
		DEBUG_MSG("\t(" << baseGraph_.id(baseGraph_.source(a)) << ", " 
			<< baseGraph_.id(baseGraph_.target(a)) << "): " << flowMap_[a]);
	}
}

int FlowGraph::sumOutFlow(Node n) const
{
	int flow = 0;
	for(Graph::OutArcIt oa(baseGraph_, n); oa != lemon::INVALID; ++oa)
		flow += flowMap_[oa];
	return flow;
}

int FlowGraph::sumInFlow(Node n) const
{
	int flow = 0;
	for(Graph::InArcIt ia(baseGraph_, n); ia != lemon::INVALID; ++ia)
		flow += flowMap_[ia];
	return flow;
}

} // end namespace dpct