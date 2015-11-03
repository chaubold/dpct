#include "flowgraph.h"
#include "log.h"

#include <chrono>
#include <assert.h>

namespace dpct
{
FlowGraph::FlowGraph():
	arcEnabledMap_(baseGraph_),
	flowMap_(baseGraph_),
	capacityMap_(baseGraph_),
	filteredGraph_(baseGraph_, arcEnabledMap_)
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
	arcEnabledMap_[a] = true;
	return a;
}

/// create duplicated parent node for the given node with the given cost
FlowGraph::Arc FlowGraph::allowMitosis(FlowGraph::Node parent,
	double divisionCost)
{
	// set up duplicate with disabled in arc
	Node duplicate = addNode(nodeCosts_[parent]);
	Arc a = addArc(source_, duplicate, {divisionCost});
	arcEnabledMap_[a] = false;

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
	std::chrono::time_point<std::chrono::high_resolution_clock> startTime_ = std::chrono::high_resolution_clock::now();
	ShortestPathResult result;
	do
	{
		LOG_MSG("\t>>> Iteration");
		LOG_MSG("Current Flow:");
		printAllFlows();

		std::shared_ptr<ResidualGraph> residualGraph = createResidualGraph();
		std::shared_ptr<ResidualDistMap> residualDistMap = createResidualDistanceMap(residualGraph);
		result = findShortestPath(residualGraph, residualDistMap);

		LOG_MSG("\tFound " << (std::get<1>(result) ? "path": "cycle")
				<< " of length " << std::get<0>(result).size() 
				<< " of distance " << std::get<2>(result));

		if(std::get<0>(result).size() > 0 && std::get<2>(result) < 0.0)
		{
#ifdef DEBUG_LOG
			printPath(std::get<0>(result)); std::cout << std::endl;
#endif
			augmentUnitFlow(std::get<0>(result), residualGraph);
			updateEnabledArcs(std::get<0>(result), residualGraph);
		}
	}
	while(std::get<0>(result).size() > 0 && std::get<2>(result) < 0.0);

	cleanUpDuplicatedOutArcs();

	std::chrono::time_point<std::chrono::high_resolution_clock> endTime_ = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed_seconds = endTime_ - startTime_;
	LOG_MSG("Tracking took " << elapsed_seconds.count() << " secs");
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
std::shared_ptr<FlowGraph::ResidualGraph> FlowGraph::createResidualGraph()
{
	filteredGraph_ = FilteredGraph(baseGraph_, arcEnabledMap_);
	std::shared_ptr<ResidualGraph> residualGraph = std::make_shared<ResidualGraph>(filteredGraph_, capacityMap_, flowMap_);

    return residualGraph;
}

std::shared_ptr<FlowGraph::ResidualDistMap> FlowGraph::createResidualDistanceMap(std::shared_ptr<ResidualGraph> residualGraphPtr)
{
	ResidualGraph& residualGraph = *residualGraphPtr;
	std::shared_ptr<ResidualDistMap> residualDistMapPtr = std::make_shared<ResidualDistMap>(residualGraph);
	ResidualDistMap& residualDistMap = *residualDistMapPtr;
	for(ResidualGraph::ArcIt a(residualGraph); a != lemon::INVALID; ++a)
    {
        if(residualGraph.forward(a))
        {
        	Arc orig = lemon::findArc(baseGraph_, residualGraph.source(a), residualGraph.target(a));
        	int origFlow = flowMap_[orig];
        	if(baseGraph_.target(orig) == target_)
            	residualDistMap[a] = arcCosts_[orig][origFlow];
        	else
        		residualDistMap[a] = arcCosts_[orig][origFlow] + nodeCosts_[baseGraph_.target(orig)][origFlow];
        }
        else
        {
            Arc orig = lemon::findArc(baseGraph_, residualGraph.target(a), residualGraph.source(a));
            int origFlow = flowMap_[orig] - 1;
            assert(origFlow >= 0);

            if(baseGraph_.target(orig) == target_)
            	residualDistMap[a] = -1.0 * arcCosts_[orig][origFlow];
        	else
        		residualDistMap[a] = -1.0 * (arcCosts_[orig][origFlow] + nodeCosts_[baseGraph_.target(orig)][origFlow]);
        }
    }
    return residualDistMapPtr;
}

/// find a shortest path (bool=true) or a negative cost cycle (bool=false)
FlowGraph::ShortestPathResult FlowGraph::findShortestPath(std::shared_ptr<ResidualGraph> residualGraph, std::shared_ptr<ResidualDistMap> residualDistMap)
{
    BellmanFord bf(*residualGraph, *residualDistMap);
    bf.init();
    bf.addSource(source_);

	Path p;
	double pathCost = 0.0;

    if(bf.checkedStart())
    {	
    	// found path
        if(bf.reached(target_))
        {
        	pathCost = bf.dist(target_);
        	for(ResidualGraph::Arc a = bf.predArc(target_); a != lemon::INVALID; a = bf.predArc(residualGraph->source(a)))
            {
                p.push_back(a);
            }
        }

        return std::make_tuple(p, true, pathCost);
    }
    else
    {
    	// found cycle
    	lemon::Path<ResidualGraph> path = bf.negativeCycle();
    	for(lemon::Path<ResidualGraph>::ArcIt a(path); a != lemon::INVALID; ++a)
        {
            p.push_back(a);
        }
        // cycles are always negative if they are found like this, but value needs to be computed manually!
    	return std::make_tuple(p, false, -1.0);
    }
}

void FlowGraph::printPath(const Path& p)
{
	for(auto a : p)
    {
        std::cout << "(" << baseGraph_.id(baseGraph_.target(a)) << ", " 
        		<< baseGraph_.id(baseGraph_.source(a)) << ") ";
    }
}

/// augment flow along a path or cycle, adding one unit of flow forward, and subtracting one backwards
void FlowGraph::augmentUnitFlow(const FlowGraph::Path& p, std::shared_ptr<ResidualGraph> residualGraph)
{
	for(auto a : p)
	{
		int delta = (residualGraph->forward(a) ? 1 : -1);
		Arc orig = lemon::findArc(baseGraph_, residualGraph->source(a), residualGraph->target(a));
		flowMap_[orig] += delta;
	}
}

/// updates the arcEnabled map by checking which divisions should be enabled/disabled after this track
void FlowGraph::updateEnabledArcs(const FlowGraph::Path& p, std::shared_ptr<ResidualGraph> residualGraph)
{
	// define functions for enabling / disabling
	auto toggleOutArcs = [&](Node n, bool state)
	{
		DEBUG_MSG("Setting out arcs of " << (baseGraph_.id(n)) << " to " << (state?"true":"false"));
		for(Graph::OutArcIt oa(baseGraph_, n); oa != lemon::INVALID; ++oa)
			arcEnabledMap_[oa] = state;
	};

	auto toggleInArcs = [&](Node n, bool state)
	{
		DEBUG_MSG("Setting in arcs of " << baseGraph_.id(n) << " to " << (state?"true":"false"));
		for(Graph::InArcIt ia(baseGraph_, n); ia != lemon::INVALID; ++ia)
			arcEnabledMap_[ia] = state;
	};

	auto toggleOutArcsBut = [&](Node n, Node exception, bool state)
	{
		DEBUG_MSG("Setting out arcs of " << baseGraph_.id(n) 
			<< " but " << baseGraph_.id(exception) 
			<< " to " << (state?"true":"false"));
		for(Graph::OutArcIt oa(baseGraph_, n); oa != lemon::INVALID; ++oa)
		{
			if(baseGraph_.target(oa) != exception)
				arcEnabledMap_[oa] = state;
		}
	};

	auto toggleInArcsBut = [&](Node n, Node exception, bool state)
	{
		DEBUG_MSG("Setting in arcs of " << baseGraph_.id(n) 
			<< " but " << baseGraph_.id(exception)
			<< " to " << (state?"true":"false"));
		for(Graph::InArcIt ia(baseGraph_, n); ia != lemon::INVALID; ++ia)
		{
			if(baseGraph_.source(ia) != exception)
				arcEnabledMap_[ia] = state;
		}
	};

	auto toggleDivision = [&](Node div, Node target, bool divState)
	{
		DEBUG_MSG("Setting division " << baseGraph_.id(div) << " to " << (divState?"true":"false"));
		for(Graph::InArcIt ia(baseGraph_, div); ia != lemon::INVALID; ++ia)
		{
			DEBUG_MSG("\ttoggling division arc " << baseGraph_.id(baseGraph_.source(ia)) << ", " << baseGraph_.id(baseGraph_.target(ia)));
			arcEnabledMap_[ia] = divState;
		}

		for(Graph::OutArcIt oa(baseGraph_, div); oa != lemon::INVALID; ++oa)
		{
			if(baseGraph_.target(oa) == target)
			{
				DEBUG_MSG("\ttoggling move arc " << baseGraph_.id(baseGraph_.source(oa)) << ", " << baseGraph_.id(baseGraph_.target(oa)));
				arcEnabledMap_[oa] = !divState;
				return;
			}
		}
		DEBUG_MSG("was not able to find the arc to " << baseGraph_.id(target) << "!");
	};

	auto toggleAppearanceArc = [&](Node n, bool state)
	{
		for(Graph::InArcIt ia(baseGraph_, n); ia != lemon::INVALID; ++ia)
		{
			if(baseGraph_.source(ia) == source_)
			{
				DEBUG_MSG("Setting appearance of " << baseGraph_.id(n) << " to " << (state?"true":"false"));
				arcEnabledMap_[ia] = state;
				return;
			}
		}
		DEBUG_MSG("Didn't find appearance arc of " << baseGraph_.id(n));
	};

	auto toggleDisappearanceArc = [&](Node n, bool state)
	{
		for(Graph::OutArcIt oa(baseGraph_, n); oa != lemon::INVALID; ++oa)
		{
			if(baseGraph_.target(oa) == target_)
			{
				DEBUG_MSG("Setting disappearance of " << baseGraph_.id(n) << " to " << (state?"true":"false"));
				arcEnabledMap_[oa] = state;
				return;
			}
		}
		DEBUG_MSG("Didn't find disappearance arc of " << baseGraph_.id(n));
	};

	// check all arcs on path whether they toggle other arc states
	for(auto a : p)
	{
		bool forward = residualGraph->forward(a);
		Node source = forward ? residualGraph->source(a) : residualGraph->target(a);
		Node target = forward ? residualGraph->target(a) : residualGraph->source(a);
		DEBUG_MSG("Updating stuff for " << (forward? "forward" : "backward") << " edge from " 
			<< residualGraph->id(source) << "=" << baseGraph_.id(source) << " to "
			<< residualGraph->id(target) << "=" << baseGraph_.id(target));

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