#ifndef DPCT_FLOWGRAPH_H
#define DPCT_FLOWGRAPH_H

#include <lemon/adaptors.h>
#include <lemon/bellman_ford.h>
#include <lemon/list_graph.h>
#include <map>
#include <tuple>
#include <memory>

#include "userdata.h"

namespace dpct
{

/**
 * A flow graph manages nodes and arcs with attached costs, capacities and flow
 */
class FlowGraph {
public: // typedefs
	typedef lemon::ListDigraph Graph;
    typedef Graph::Node Node;
    typedef Graph::Arc Arc;
    typedef Graph::ArcMap<double> DistMap;
    typedef Graph::ArcMap<int> FlowMap;
    typedef Graph::ArcMap<int> CapacityMap;
    typedef Graph::ArcMap<bool> ArcEnabledMap;
    typedef lemon::FilterArcs<Graph> FilteredGraph;
    typedef lemon::ResidualDigraph< FilteredGraph, CapacityMap, FlowMap > ResidualGraph;
    typedef ResidualGraph::ArcMap<double> ResidualDistMap;
    typedef lemon::BellmanFord<ResidualGraph, ResidualDistMap> BellmanFord;
    typedef std::vector<double> CostVector;
    typedef std::vector<ResidualGraph::Arc> Path;
    typedef std::tuple<Path, bool, double> ShortestPathResult;

public: // API
	FlowGraph();

	Node addNode(const CostVector& costs);

	Arc addArc(Node source,
		Node target,
		const CostVector& costs);

	/// create duplicated parent node for the given node with the given cost
	/// ATTENTION: needs to be called AFTER adding all other arcs or it will copy too few
	Arc allowMitosis(Node parent, double divisionCost);

	/// start the tracking
	void maxFlowMinCostTracking();

	Node getSource() const { return source_; }
	Node getTarget() const { return target_; }
	int sumOutFlow(Node n) const;
	int sumInFlow(Node n) const;

	const FlowMap& getFlowMap() const { return flowMap_; }

private:
	/// use the current flow and arcEnabled maps to create a residual graph using the appropriate cost deltas
	std::shared_ptr<ResidualGraph> createResidualGraph();

	std::shared_ptr<ResidualDistMap> createResidualDistanceMap(std::shared_ptr<ResidualGraph> residualGraph);

	/// find a shortest path (bool=true) or a negative cost cycle (bool=false)
	ShortestPathResult findShortestPath(std::shared_ptr<ResidualGraph> residualGraph, 
		std::shared_ptr<ResidualDistMap> residualDistMap);

	/// augment flow along a path or cycle, adding one unit of flow forward, and subtracting one backwards
	void augmentUnitFlow(const Path& p, std::shared_ptr<ResidualGraph> residualGraph);

	/// updates the arcEnabled map by checking which divisions should be enabled/disabled after this track
	/// ATTENTION: assumes flow has been augmented for this path already!
	void updateEnabledArcs(const Path& p, std::shared_ptr<ResidualGraph> residualGraph);

	/// copy flow from the duplicated to the original arcs so it is seen by the user
	void cleanUpDuplicatedOutArcs();

	void printPath(const Path& p);
	void printAllFlows();
private:
	/// graph containing all nodes and arcs and duplicated parents of divisions
	Graph baseGraph_;
	Node source_;
	Node target_;

	/// depending on current flow
	ArcEnabledMap arcEnabledMap_;

	/// current filtered graph
	FilteredGraph filteredGraph_;

	/// current arc flow
	FlowMap flowMap_;

	/// capacities of arcs
	CapacityMap capacityMap_;

	/// store cost of arcs and nodes
	std::map<Node, CostVector> nodeCosts_;
	std::map<Arc, CostVector> arcCosts_;

	/// mapping between parent and duplicated parent nodes
	std::map<Node, Node> parentToDuplicateMap_;
	std::map<Node, Node> duplicateToParentMap_;
};

} // end namespace dpct

#endif