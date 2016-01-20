#ifndef DPCT_FLOWGRAPH_H
#define DPCT_FLOWGRAPH_H

#include <lemon/adaptors.h>
#include <lemon/bellman_ford.h>
#include <lemon/list_graph.h>
#include <map>
#include <tuple>
#include <memory>
#include <chrono>

#include "residualgraph.h"

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
	/// create residual graph and set up all arc flows etc
	void initializeResidualGraph();

	/// augment flow along a path or cycle, adding one unit of flow forward, and subtracting one backwards
	void augmentUnitFlow(const Path& p);

	/// updates the arcEnabled map by checking which divisions should be enabled/disabled after this track
	/// ATTENTION: assumes flow has been augmented for this path already!
	void updateEnabledArcs(const Path& p);

	/// enable an arc according to our division / appearance / disappearance constraints
	void enableArc(const Arc& a, bool state);

	/// update capacity and cost of residual graph arc
	void updateArc(const Arc& a);

	/// copy flow from the duplicated (division) to the original arcs so it is seen by the user
	void cleanUpDuplicatedOutArcs();

	double getNodeCost(const Node& n, int flow);
	double getArcCost(const Arc& a, int flow);

	void printPath(const Path& p);
	void printAllFlows();
private:
	/// graph containing all nodes and arcs and duplicated parents of divisions
	Graph baseGraph_;
	Node source_;
	Node target_;

	/// residual graph
	std::shared_ptr<ResidualGraph> residualGraph_;

	/// current arc flow
	FlowMap flowMap_;

	/// capacities of arcs
	CapacityMap capacityMap_;

	/// store cost of arcs and nodes
	NodeCostMap nodeCosts_;
	ArcCostMap arcCosts_;

	/// mapping between parent and duplicated parent nodes
	std::map<Node, Node> parentToDuplicateMap_;
	std::map<Node, Node> duplicateToParentMap_;
};

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