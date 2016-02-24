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
	nodeTimestepMap_[source_] = 0;
	target_ = baseGraph_.addNode();
	nodeTimestepMap_[target_] = 1;
}

FlowGraph::FullNode FlowGraph::addNode(const CostVector& costs, size_t timestep)
{
	assert(costs.size() > 0);
	
	FullNode f;
	f.u = baseGraph_.addNode();
	f.v = baseGraph_.addNode();
	f.a = addArc(f.u, f.v, costs);
	intermediateArcs_.insert(f.a);
	nodeTimestepMap_[f.u] = timestep * 2 + 1;
	nodeTimestepMap_[f.v] = timestep * 2 + 2;
	if(timestep * 2 + 2 >= nodeTimestepMap_[target_])
		nodeTimestepMap_[target_] = timestep * 2 + 3;

	return f;
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

FlowGraph::Arc FlowGraph::addArc(FlowGraph::FullNode source,
	FlowGraph::FullNode target,
	const CostVector& costs)
{
	return addArc(source.v, target.u, costs);
}

/// create duplicated parent node for the given node with the given cost
FlowGraph::Arc FlowGraph::allowMitosis(FlowGraph::FullNode parent,
	double divisionCost)
{
	// set up duplicate with disabled in arc
	Node duplicate = baseGraph_.addNode();
	nodeTimestepMap_[duplicate] = nodeTimestepMap_[parent.v];
	Arc a = addArc(source_, duplicate, {divisionCost});

	// copy all out arcs, but with capacity=1 only, and don't add disappearance arc
	for(Graph::OutArcIt oa(baseGraph_, parent.v); oa != lemon::INVALID; ++oa)
	{
		if(baseGraph_.target(oa) != target_)
			addArc(duplicate, baseGraph_.target(oa), {arcCosts_[oa][0]});
	}

	parentToDuplicateMap_[parent.v] = duplicate;
	duplicateToParentMap_[duplicate] = parent.v;

	return a;
}

/// start the tracking
double FlowGraph::maxFlowMinCostTracking(
	double initialStateEnergy, 
	bool useBackArcs, 
	size_t maxNumPaths, 
	bool useOrderedNodeListInBF)
{
	TimePoint startTime_ = std::chrono::high_resolution_clock::now();

	initializeResidualGraph(useBackArcs, useOrderedNodeListInBF);

	ResidualGraph::ShortestPathResult result;
	size_t iter=0;
	double currentEnergy = initialStateEnergy;
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

		if(result.second > -0.00000001)
		{
			printPath(result.first); std::cout << std::endl;
			break;
		}

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
			currentEnergy += result.second; // decrease energy
		}
		TimePoint iterationEndTime = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed_seconds = iterationEndTime - iterationStartTime;
		LOG_MSG("\t<<<Iteration " << iter << " done in " << elapsed_seconds.count() 
				<< " secs, system Energy=" << currentEnergy);
		iter++;
	}
	while(result.first.size() > 0 && result.second < 0.0 && (maxNumPaths < 1 || iter < maxNumPaths));

	TimePoint endTime_ = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed_seconds = endTime_ - startTime_;
	LOG_MSG("Tracking took " << elapsed_seconds.count() << " secs and " << iter << " iterations");
	LOG_MSG("Final energy: " << currentEnergy);
	return currentEnergy;
}

void FlowGraph::initializeResidualGraph(bool useBackArcs, bool useOrderedNodeListInBF)
{
	std::cout << "Initializing Residual Graph ..." << std::flush;
	residualGraph_ = std::make_shared<ResidualGraph>(baseGraph_, nodeTimestepMap_, useBackArcs, useOrderedNodeListInBF);
	
	for(Graph::ArcIt a(baseGraph_); a != lemon::INVALID; ++a)
    {
    	updateArc(a);
    	enableArc(a, true);

    	// division arcs are disabled at the beginning.
    	// Tokens are provided on division forward arcs, and forbidden on the corresponding mother backward arc
    	if(duplicateToParentMap_.find(baseGraph_.target(a)) != duplicateToParentMap_.end())
    	{
    		residualGraph_->addProvidedToken(a, ResidualGraph::Forward, 
    										 baseGraph_.id(duplicateToParentMap_[baseGraph_.target(a)]));
    	}
    	else if(parentToDuplicateMap_.find(baseGraph_.target(a)) != parentToDuplicateMap_.end())
    	{
    		residualGraph_->addForbiddenToken(a, ResidualGraph::Backward, baseGraph_.id(baseGraph_.target(a)));
    	}
    }

    // enable all arcs depending on their flow
    for(Graph::ArcIt a(baseGraph_); a != lemon::INVALID; ++a)
    {
    	updateEnabledArc(a);
    }
    std::cout << " done" << std::endl;
}

void FlowGraph::printPath(const Path& p)
{
	for(auto a : p)
    {
        std::cout << "(" << baseGraph_.id(baseGraph_.target(a.first)) << ", " 
        		<< baseGraph_.id(baseGraph_.source(a.first)) << "):" << a.second << " ";
    }
}

void FlowGraph::synchronizeDivisionDuplicateArcFlows()
{
	for(auto coupledNodeIt : parentToDuplicateMap_)
	{
		for(Graph::OutArcIt oa(baseGraph_, coupledNodeIt.first); oa != lemon::INVALID; ++oa)
		{
			int origFlow = flowMap_[oa];

			for(Graph::OutArcIt duplicateOa(baseGraph_, coupledNodeIt.second); duplicateOa != lemon::INVALID; ++duplicateOa)
			{
				if(baseGraph_.target(oa) == baseGraph_.target(duplicateOa))
				{
					int duplicateFlow = flowMap_[duplicateOa];
					int flow = std::max(origFlow, duplicateFlow);

					flowMap_[duplicateOa] = std::min(flow, int(1));
					flowMap_[oa] = flow;
					break;
				}
			}
		}
	}
}

/// augment flow along a path or cycle, adding one unit of flow forward, and subtracting one backwards
void FlowGraph::augmentUnitFlow(const FlowGraph::Path& p)
{
	for(const std::pair<Arc, int>& af : p)
	{
		flowMap_[af.first] += af.second;
		updateArc(af.first);

		// see if this is a division duplicate pair, if yes, keep arc flows synchronized
		auto coupledNodeIt = duplicateToParentMap_.find(baseGraph_.source(af.first));
		if(coupledNodeIt != duplicateToParentMap_.end())
		{
			for(Graph::OutArcIt oa(baseGraph_, coupledNodeIt->second); oa != lemon::INVALID; ++oa)
			{
				if(baseGraph_.target(oa) == baseGraph_.target(af.first))
				{
					flowMap_[oa] += af.second;
					updateArc(oa);
					break;
				}
			}
			continue;
		}

		coupledNodeIt = parentToDuplicateMap_.find(baseGraph_.source(af.first));
		if(coupledNodeIt != parentToDuplicateMap_.end())
		{
			for(Graph::OutArcIt oa(baseGraph_, coupledNodeIt->second); oa != lemon::INVALID; ++oa)
			{
				if(baseGraph_.target(oa) == baseGraph_.target(af.first))
				{
					// set the duplicated arcs, but don't exceed their capacity!
					flowMap_[oa] = std::min(flowMap_[af.first], 1);
					updateArc(oa);
					break;
				}
			}
		}
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

void FlowGraph::updateArc(const Arc& a)
{
	int flow = flowMap_[a];
	DEBUG_MSG("Found " << flow << " flow along arc " << baseGraph_.id(baseGraph_.source(a)) << "->" << baseGraph_.id(baseGraph_.target(a)));
	int capacity = capacityMap_[a];
	if(flow < 0)
		throw std::runtime_error("Found Arc with negative flow!");
	if(flow > capacity)
		throw std::runtime_error("Found Arc with more flow than capacity!");

	// forward arc:
	double forwardCost = getArcCost(a, flow);
    residualGraph_->updateArc(a, ResidualGraph::Forward, forwardCost, capacity - flow);

    // backward arc:
    double backwardCost = -1.0 * getArcCost(a, flow-1);
    residualGraph_->updateArc(a, ResidualGraph::Backward, backwardCost, flow);
}

/// updates the enabled arcs in the residual graph by checking 
/// which divisions should be enabled/disabled after this track
void FlowGraph::updateEnabledArcs(const FlowGraph::Path& p)
{
	for(const std::pair<Arc, int>& af : p)
	{
		bool forward = af.second > 0;
		Node source = baseGraph_.source(af.first);
		Node target = baseGraph_.target(af.first);
		DEBUG_MSG("Updating stuff for " << (forward? "forward" : "backward") << " edge from " 
			<< baseGraph_.id(source) << " to " << baseGraph_.id(target));

		updateEnabledArc(af.first);
	}
}

void FlowGraph::updateEnabledArc(const FlowGraph::Arc& a)
{
	Node source = baseGraph_.source(a);
	Node target = baseGraph_.target(a);

	DEBUG_MSG("Updating stuff for" << " edge from " 
		<< baseGraph_.id(source) << " to " << baseGraph_.id(target));

	// division updates: enable if mother cell is used exactly once, but flow is not disappearing
	if(parentToDuplicateMap_.find(source) != parentToDuplicateMap_.end() && target != target_)
	{
		if(sumInFlow(source) == 1)
		{
			DEBUG_MSG("Enabling division of " << baseGraph_.id(source));
			// we have exactly one unit of flow forward through a parent node -> allows division
			toggleDivision(parentToDuplicateMap_[source], target, true);
		}
		else
		{
			DEBUG_MSG("Disabling division of " << baseGraph_.id(source));
			// in all other cases, a division is not allowed
			toggleDivision(parentToDuplicateMap_[source], target, false);
		}
	}
	// division used/unused -> toggle mother cell's in and out arcs
	else if(duplicateToParentMap_.find(target) != duplicateToParentMap_.end())
	{
		if(flowMap_[a] == 1)
		{
			// adding flow through division -> parent cannot be undone
			toggleInArcs(duplicateToParentMap_[target], false);
			restrictOutArcCapacity(duplicateToParentMap_[target], true);
		}
		else
		{
			// removing flow from division -> parent can be undone again
			// FIXME: but not disappearance!
			toggleInArcs(duplicateToParentMap_[target], true);
			restrictOutArcCapacity(duplicateToParentMap_[target], false);
		}
	}

	// forbid partial appearance/disappearance
	if(source == source_)
	{
		// changing usage of appearance arc, enable/disable all other incomings to target
		toggleInArcsBut(target, source, flowMap_[a] == 0);
	}
	else if(target == target_)
	{
		// changing usage of disappearance arc, enable/disable all other outgoings of source
		toggleOutArcsBut(source, target, flowMap_[a] == 0);
	}
	else if(intermediateArcs_.count(a) == 0)
	{
		// we did not use an appearance or disappearance arc! 
		// enable those if no other in-/out- flow at that arc yet
		toggleDisappearanceArc(source, sumOutFlow(source) == 0);
		toggleAppearanceArc(target, sumInFlow(target) == 0);
	}

	// TODO: exclusion constraints
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