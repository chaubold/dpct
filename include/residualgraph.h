#ifndef DPCT_RESIDUALGRAPH_H
#define DPCT_RESIDUALGRAPH_H

#include <lemon/list_graph.h>
#include <lemon/bellman_ford.h>
#include <map>
#include <memory>
#include <assert.h>

namespace dpct
{

/**
 * A residual graph has one forward an one backward arc for an original graph, 
 * with different costs, and only enabling them when the flow of the original graph allows them to be used.
 * This is a replacement for Lemon's ResidualGraph, as using several graph adapters on top of each other
 * slows down the shortest path search incredibly.
 */
class ResidualGraph : public lemon::ListDigraph {
public: // typedefs
	typedef lemon::ListDigraph Graph;
    typedef Graph::Node Node;
    typedef Graph::Node OriginalNode;
    typedef Graph::Arc Arc;
    typedef Graph::Arc OriginalArc;
    typedef Graph::ArcMap<double> DistMap;
    typedef Graph::ArcMap<int> FlowMap;
    typedef Graph::ArcMap<bool> ArcEnabledMap;
    typedef std::map<Node, Node> OriginMap;
    typedef std::map<Node, Node> ResidualNodeMap;
    typedef lemon::BellmanFord<ResidualGraph, DistMap> BellmanFord;
    typedef std::vector< std::pair<OriginalArc, int> > Path; // combines arc with flow delta (direction)
    typedef std::pair<Path, double> ShortestPathResult;
    typedef std::pair<Node, Node> ResidualArcCandidate;

	static const bool Forward = true;
	static const bool Backward = false;

public: // API
	ResidualGraph(const Graph& original);
	
	/// set arc cost for the residual forward/backward arc corresponding to a in the original graph
	/// if capacity = 0, the arc will be disabled and the cost ignored
	void updateArc(const OriginalArc& a, bool forward, double cost, int capacity);

	/// enable / disable both residual arcs of the original arc
	void enableArc(const OriginalArc& a, bool state);

	/// save graph to dot file
	void toDot(const std::string& filename, const Path& p);

	/// find the shortest augmenting path/circle (if any)
	ShortestPathResult findShortestPath(
		const OriginalNode& origSource, 
		const OriginalNode& origTarget) const;

private:
	/// enable / disable residual arc
	void enableArc(const ResidualArcCandidate& a, bool state);

	/// set arc cost for the residual arc
	void updateResidualArc(const ResidualArcCandidate& a, double cost, int capacity);

	/**
	 * @brief Transform original graph arc to ordered residual graph node pair, using the specified direction
	 * @param a original graph arc
	 * @param forward true if we want the forward residual arc along this edge
	 * 
	 * @return residual graph node pair 
	 */
	ResidualArcCandidate undirectedArcToPair(const OriginalArc& a, bool forward) const
	{
		return forward ? arcToPair(a) : arcToInversePair(a);
	}

	/**
	 * @brief Transform an arc of the original graph into an forward oriented node pair (the ResidualArcCandidate) of the residual graph
	 * @param a original graph arc
	 * 
	 * @return forward arc in residual graph encoded as pair of nodes
	 */
	ResidualArcCandidate arcToPair(const OriginalArc& a) const
	{
		return std::make_pair(residualNodeMap_.at(originalGraph_.source(a)), 
				residualNodeMap_.at(originalGraph_.target(a)));
	}

	/**
	 * @brief Transform an arc of the original graph into an backward oriented node pair (the ResidualArcCandidate) of the residual graph
	 * @param a original graph arc
	 * 
	 * @return backward arc in residual graph encoded as pair of nodes
	 */
	ResidualArcCandidate arcToInversePair(const OriginalArc& a) const
	{
		return std::make_pair(residualNodeMap_.at(originalGraph_.target(a)), 
				residualNodeMap_.at(originalGraph_.source(a)));
	}

	/**
	 * @brief Transform an arc of the residual graph into an ordered node pair (the ResidualArcCandidate) of the residual graph
	 * @param a residual graph arc
	 * 
	 * @return arc in residual graph encoded as pair of nodes
	 */
	ResidualArcCandidate residualArcToPair(const Arc& a) const
	{
		return std::make_pair(source(a), target(a));
	}

	/**
	 * @brief Transform a node pair of the residual graph into an residual arc
	 * @param a residual graph node pair
	 * 
	 * @return arc in residual graph
	 */
	Arc pairToResidualArc(const ResidualArcCandidate& ac) const
	{
		return lemon::findArc(*this, ac.first, ac.second);
	}

	// return the original arc and a boolean whether the residual arc was forward or backward
	std::pair<OriginalArc, bool> pairToOriginalArc(const ResidualArcCandidate& ac) const
	{
		OriginalArc a = lemon::findArc(originalGraph_, originMap_.at(ac.first), originMap_.at(ac.second));
		if(a != lemon::INVALID)
			return std::make_pair(a, true);
		else
		{
			a = lemon::findArc(originalGraph_, originMap_.at(ac.second), originMap_.at(ac.first));
			assert(a != lemon::INVALID);
			return std::make_pair(a, false);
		}
	}

private:
	/// Original graph
	const Graph& originalGraph_;

	/// store cost of arcs, independent of whether they are enabled or not
	std::map<ResidualArcCandidate, double> residualArcCost_;

	/// last state of the arc depending on the flow values of the original graph.
	/// if enableArc(graph,arc,false) is called, this map is not touched!
	std::map<ResidualArcCandidate, bool> residualArcPresent_;

	/// the distance(=cost) map of this residual graph used for shortest path computation
	DistMap residualDistMap_;

	/// a mapping from all residual nodes to original nodes
	OriginMap originMap_;

	/// a mapping from original nodes to residual nodes
	ResidualNodeMap residualNodeMap_;
};

} // end namespace dpct

#endif