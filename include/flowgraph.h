#ifndef DPCT_FLOWGRAPH_H
#define DPCT_FLOWGRAPH_H

#include <lemon/adaptors.h>
#include <lemon/bellman_ford.h>
#include <lemon/list_graph.h>
#include <map>
#include <set>
#include <tuple>
#include <memory>
#include <chrono>

#include "residualgraph.h"
#include "log.h"

namespace dpct
{

/**
 * A flow graph manages nodes and arcs with attached costs, capacities and flow. 
 * It can run min cost max flow tracking based on the bellman ford shortest path algorithm,
 * but alters the residual graph in each iteration such that it obeys all consistency constraints.
 */
class FlowGraph {
public: // typedefs
	typedef lemon::ListDigraph Graph;
    typedef Graph::Node Node;
    typedef Graph::Arc Arc;
    struct FullNode{
    	Node u;
    	Node v;
    	Arc a;
    };
    typedef Graph::ArcMap<double> DistMap;
    typedef Graph::ArcMap<int> FlowMap;
    typedef Graph::ArcMap<int> CapacityMap;
    typedef std::vector<double> CostVector;
    typedef std::map<Node, CostVector> NodeCostMap;
	typedef std::map<Arc, CostVector> ArcCostMap;
	typedef std::vector< std::pair<Arc, int> > Path;
	typedef std::chrono::time_point<std::chrono::high_resolution_clock> TimePoint;

public: // API
	FlowGraph();

	FullNode addNode(const CostVector& costs, size_t timestep=0);

	Arc addArc(Node source, Node target, const CostVector& costs);
	Arc addArc(FullNode source, FullNode target, const CostVector& costs);

	/// create duplicated parent node for the given node with the given cost
	/// ATTENTION: needs to be called AFTER adding all other arcs or it will copy too few
	Arc allowMitosis(FullNode parent, double divisionCost);

	/**
	 * @brief run tracking by finding sugmenting shortest paths in the residual graph
	 * @param initialStateEnergy the energy of the system if no objects are sent through (state 0)
	 * @param useBackArcs whether the residual graph should contain the edges that allow backward flow
	 * @param maxNumPaths if >0 this  limits the number of augmenting shortest paths that should be found
	 */
	double maxFlowMinCostTracking(
		double initialStateEnergy=0.0, 
		bool useBackArcs=true, 
		size_t maxNumPaths=0,
		bool useOrderedNodeListInBF=true);

	Node getSource() const { return source_; }
	Node getTarget(size_t index=0);
	bool isTarget(Node t) const;
	int sumOutFlow(Node n) const;
	int sumInFlow(Node n) const;

	FlowMap& getFlowMap() { return flowMap_; }

	/// get the graph (used in test)
	Graph& getGraph() { return baseGraph_; }

	/// augment flow along a path or cycle, adding one unit of flow forward, and subtracting one backwards
	void augmentUnitFlow(const Path& p);

	/// updates the arc availability in residual graph for each arc on the path
	/// by checking which divisions should be enabled/disabled after this track
	/// ATTENTION: assumes flow has been augmented for this path already!
	void updateEnabledArcs(const Path& p);

	/**
	 * @brief when specifying a flow map from the outside, call this method to make sure flows
	 *        of out arcs of duplicated division nodes are in sync with those of the parent
	 */
	void synchronizeDivisionDuplicateArcFlows();

	/// create residual graph and set up all arc flows etc
	void initializeResidualGraph(bool useBackArcs, bool useOrderedNodeListInBF);

private:
	/// updates the arc availability in residual graph for this arc
	/// by checking which divisions should be enabled/disabled after this track
	/// ATTENTION: assumes flow has been augmented for this path already!
	void updateEnabledArc(const Arc& a);

	/// enable an arc according to our division / appearance / disappearance constraints
	void enableArc(const Arc& a, bool state);

	/// update capacity and cost of residual graph arc
	void updateArc(const Arc& a);

	double getArcCost(const Arc& a, int flow);
	
	void printPath(const Path& p);
	void printAllFlows();

	// define functions for enabling / disabling
	void toggleOutArcs(const Node& n, bool state);
	void toggleInArcs(const Node& n, bool state);
	void toggleOutArcsBut(const Node& n, const Node& exception, bool state);
	void toggleOutArcsButTarget(const Node& n, bool state);
	void toggleInArcsBut(const Node& n, const Node& exception, bool state);
	void toggleDivision(const Node& div, const Node& target, bool divState);
	void toggleAppearanceArc(const Node& n, bool state);
	void toggleDisappearanceArc(const Node& n, bool state);
	void restrictOutArcCapacity(const Node& n, bool state);

private:
	/// graph containing all nodes and arcs and duplicated parents of divisions
	Graph baseGraph_;
	Node source_;
	std::vector<Node> targets_;

	/// residual graph
	std::shared_ptr<ResidualGraph> residualGraph_;

	/// current arc flow
	FlowMap flowMap_;

	/// capacities of arcs
	CapacityMap capacityMap_;

	/// store cost of arcs and nodes
	ArcCostMap arcCosts_;

	/// mapping between parent and duplicated parent nodes
	std::map<Node, Node> parentToDuplicateMap_;
	std::map<Node, Node> duplicateToParentMap_;

	/// store a set of the arcs which are actually just used to emplace the node costs
	std::set<Arc> intermediateArcs_;

	/// store the (internal!) timestep of each node (including the duplicates)
	std::map<Node, size_t> nodeTimestepMap_;
};

// define functions for enabling / disabling
inline void FlowGraph::toggleOutArcs(const Node& n, bool state)
{
	DEBUG_MSG("Setting out arcs of " << (baseGraph_.id(n)) << " to " << (state?"true":"false"));
	for(Graph::OutArcIt oa(baseGraph_, n); oa != lemon::INVALID; ++oa)
		enableArc(oa, state);
}

inline void FlowGraph::toggleInArcs(const Node& n, bool state)
{
	DEBUG_MSG("Setting in arcs of " << baseGraph_.id(n) << " to " << (state?"true":"false"));
	for(Graph::InArcIt ia(baseGraph_, n); ia != lemon::INVALID; ++ia)
		enableArc(ia, state);
}

inline void FlowGraph::restrictOutArcCapacity(const Node& n, bool state)
{
	DEBUG_MSG("Restricting Out arc capacities of " << baseGraph_.id(n) << ": " << (state?"true":"false"));
	for(Graph::OutArcIt oa(baseGraph_, n); oa != lemon::INVALID; ++oa)
	{
		capacityMap_[oa] = (state ? 1 : arcCosts_[oa].size());
		updateArc(oa);
	}
}

inline void FlowGraph::toggleOutArcsBut(const Node& n, const Node& exception, bool state)
{
	DEBUG_MSG("Setting out arcs of " << baseGraph_.id(n) 
		<< " but " << baseGraph_.id(exception) 
		<< " to " << (state?"true":"false"));
	for(Graph::OutArcIt oa(baseGraph_, n); oa != lemon::INVALID; ++oa)
	{
		if(baseGraph_.target(oa) != exception)
			enableArc(oa, state);
	}
}

inline void FlowGraph::toggleOutArcsButTarget(const Node& n, bool state)
{
	DEBUG_MSG("Setting out arcs of " << baseGraph_.id(n) 
		<< " but " << baseGraph_.id(exception) 
		<< " to " << (state?"true":"false"));
	for(Graph::OutArcIt oa(baseGraph_, n); oa != lemon::INVALID; ++oa)
	{
		if(!isTarget(baseGraph_.target(oa)))
			enableArc(oa, state);
	}
}

inline void FlowGraph::toggleInArcsBut(const Node& n, const Node& exception, bool state)
{
	DEBUG_MSG("Setting in arcs of " << baseGraph_.id(n) 
		<< " but " << baseGraph_.id(exception)
		<< " to " << (state?"true":"false"));
	for(Graph::InArcIt ia(baseGraph_, n); ia != lemon::INVALID; ++ia)
	{
		if(baseGraph_.source(ia) != exception)
			enableArc(ia, state);
	}
}

inline void FlowGraph::toggleDivision(const Node& div, const Node& target, bool divState)
{
	DEBUG_MSG("Setting division " << baseGraph_.id(div) << " to " << (divState?"true":"false"));
	for(Graph::InArcIt ia(baseGraph_, div); ia != lemon::INVALID; ++ia)
	{
		DEBUG_MSG("\ttoggling division arc " << baseGraph_.id(baseGraph_.source(ia)) 
				  << ", " << baseGraph_.id(baseGraph_.target(ia)));
		enableArc(ia, divState);
	}

	// for(Graph::OutArcIt oa(baseGraph_, div); oa != lemon::INVALID; ++oa)
	// {
	// 	if(baseGraph_.target(oa) == target)
	// 	{
	// 		DEBUG_MSG("\ttoggling move arc " << baseGraph_.id(baseGraph_.source(oa)) 
	// 				  << ", " << baseGraph_.id(baseGraph_.target(oa)));
	// 		enableArc(oa, !divState);
	// 		return;
	// 	}
	// }
	// DEBUG_MSG("was not able to find the arc to " << baseGraph_.id(target) << "!");
}

inline void FlowGraph::toggleAppearanceArc(const Node& n, bool state)
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
}

inline void FlowGraph::toggleDisappearanceArc(const Node& n, bool state)
{
	for(Graph::OutArcIt oa(baseGraph_, n); oa != lemon::INVALID; ++oa)
	{
		if(isTarget(baseGraph_.target(oa)))
		{
			DEBUG_MSG("Setting disappearance of " << baseGraph_.id(n) << " to " << (state?"true":"false"));
			enableArc(oa, state);
			return;
		}
	}
	DEBUG_MSG("Didn't find disappearance arc of " << baseGraph_.id(n));
}

inline int FlowGraph::sumOutFlow(Node n) const
{
	int flow = 0;
	for(Graph::OutArcIt oa(baseGraph_, n); oa != lemon::INVALID; ++oa)
		flow += flowMap_[oa];
	return flow;
}

inline int FlowGraph::sumInFlow(Node n) const
{
	int flow = 0;
	for(Graph::InArcIt ia(baseGraph_, n); ia != lemon::INVALID; ++ia)
		flow += flowMap_[ia];
	return flow;
}

inline FlowGraph::Node FlowGraph::getTarget(size_t index)
{
	while(index >= targets_.size())
	{
		targets_.push_back(baseGraph_.addNode());
		nodeTimestepMap_[targets_.back()] = nodeTimestepMap_[targets_.front()];
	}

 	return targets_[index];
}

inline bool FlowGraph::isTarget(Node t) const
{
	for(auto n : targets_)
		if(t == n) return true;
	return false;
}

/**
 * @brief Output std vectors of stuff
 * 
 * @param stream output stream
 * @param feats the vector of stuff
 */
template<class T>
std::ostream& operator<<(std::ostream& stream, const std::vector<T>& feats)
{
	stream << "(";
	for(auto f_it = feats.begin(); f_it != feats.end(); ++f_it)
	{
		if(f_it != feats.begin())
			stream << ", ";
		stream << *f_it;
	}
	stream << ")";
	return stream;
}

} // end namespace dpct

#endif