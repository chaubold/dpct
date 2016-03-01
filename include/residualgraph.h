#ifndef DPCT_RESIDUALGRAPH_H
#define DPCT_RESIDUALGRAPH_H

#include <lemon/list_graph.h>
#include <lemon/maps.h>
#include "early_stopping_bellman_ford.h"
#include "log.h"
#include <map>
#include <memory>
#include <assert.h>
#include <unordered_set>
#include <set>

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
    typedef Graph::NodeMap<Arc> BfPredMap;
    typedef Graph::NodeMap<double> BfDistMap;
    typedef std::vector<Node> BfProcess;
    typedef Graph::ArcMap<int> FlowMap;
    typedef Graph::ArcMap<bool> ArcEnabledMap;
    typedef lemon::IterableValueMap<Graph, Node, size_t> NodeUpdateOrderMap;
    typedef std::map<Node, Node> OriginMap;
    typedef std::map<Node, Node> ResidualNodeMap;
    typedef size_t Token;
    typedef std::set<Token> TokenSet;
    typedef Graph::ArcMap<TokenSet> TokenSetArcMap;
    typedef lemon::EarlyStoppingBellmanFord<Graph, DistMap, Token> BellmanFord;
    typedef std::vector< std::pair<OriginalArc, int> > Path; // combines arc with flow delta (direction)
    typedef std::pair<Path, double> ShortestPathResult;
    typedef std::pair<Node, Node> ResidualArcCandidate;

	static const bool Forward = true;
	static const bool Backward = false;

public: // API
	ResidualGraph(
		const Graph& original,
		const std::map<OriginalNode, size_t>& nodeTimestepMap, 
		bool useBackArcs=true,
		bool useOrderedNodeListInBF=false);
	
	/// set arc cost for the residual forward/backward arc corresponding to a in the original graph
	/// if capacity = 0, the arc will be disabled and the cost ignored
	void updateArc(const OriginalArc& a, bool forward, double cost, int capacity);

	/// enable / disable both residual arcs of the original arc
	void enableArc(const OriginalArc& a, bool state);
	bool getArcEnabledState(const OriginalArc& a);

	/// save graph to dot file
	void fullGraphToDot(const std::string& filename, const Path& p) const;
	void toDot(const std::string& filename, const Path& p, Node& s, Node& t) const;

	/// find the shortest augmenting path/circle (if any)
	ShortestPathResult findShortestPath(
		const OriginalNode& origSource, 
		const std::vector<OriginalNode>& origTargets);

	/// configure forbidden tokens of arcs
	void addForbiddenToken(const OriginalArc& a, bool forward, Token token);
	void removeForbiddenToken(const OriginalArc& a, bool forward, Token token);

	/// configure provided tokens of arcs
	void addProvidedToken(const OriginalArc& a, bool forward, Token token);
	void removeProvidedToken(const OriginalArc& a, bool forward, Token token);

private:
	/// include/exclude an arc in this residual graph
	void includeArc(const ResidualArcCandidate& a);

	/// set arc cost for the residual arc
	void updateResidualArc(const ResidualArcCandidate& a, double cost, int capacity);

	/**
	 * @brief Check whether the path collected tokens which were forbidden on one of the later arcs
	 * 
	 * @param p path
	 * @return true if no violations were found, otherwise (false, forbiddenToken)
	 */
	std::pair<bool, Token> pathSatisfiesTokenSpecs(const Path& p) const;

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

	/// whether backwards arcs should be used at all or not (only needed for Magnusson comparison)
	bool useBackArcs_;

	/// whether we want to initialize BF's list of nodes to update in the first frame
	/// ordered by timesteps as in magnusson
	bool useOrderedNodeListInBF_;

	/// store cost of arcs, independent of whether they are enabled or not
	std::map<ResidualArcCandidate, double> residualArcCost_;

	/// last state of the arc depending on the flow values of the original graph.
	/// if enableArc(graph,arc,false) is called, this map is not touched!
	std::map<ResidualArcCandidate, bool> residualArcPresent_;

	/// whether this arc is enabled according to constraints etc
	std::map<ResidualArcCandidate, bool> residualArcEnabled_;

	/// the distance(=cost) map of this residual graph used for shortest path computation
	DistMap residualDistMap_;

	/// a mapping from all residual nodes to original nodes
	OriginMap originMap_;

	/// a mapping from original nodes to residual nodes
	ResidualNodeMap residualNodeMap_;

	/// the required and provided tokens per residual arc - persistent even if arcs are disabled (and hence removed)
	std::map<ResidualArcCandidate, TokenSet> residualArcProvidesTokens_;
	std::map<ResidualArcCandidate, TokenSet> residualArcForbidsTokens_;

	/// the actual arc maps used by BellmanFord
	TokenSetArcMap forbiddenTokenMap_;
	TokenSetArcMap providedTokenMap_;

	/// store an index for each node depending on when it should be updated
	NodeUpdateOrderMap nodeUpdateOrderMap_;

	/// create the node distance and predecessor maps only once and pass them to BF in each iteration
	BfDistMap bfDistMap_;
	BfPredMap bfPredMap_;
	BfProcess bfProcess_;
	BfProcess bfNextProcess_;
};


inline void ResidualGraph::updateArc(const OriginalArc& a, bool forward, double cost, int capacity)
{
	if(!useBackArcs_ && !forward)
		return;
	ResidualArcCandidate ac = undirectedArcToPair(a, forward);
	updateResidualArc(ac, cost, capacity);
}

inline void ResidualGraph::updateResidualArc(const ResidualArcCandidate& ac, double cost, int capacity)
{
	DEBUG_MSG("Updating residual arc (" << id(ac.first) << ", " << id(ac.second) << ") with cost " 
			<< cost << " and capacity " << capacity);
	if(capacity > 0)
	{
		residualArcCost_[ac] = cost;
		residualArcPresent_[ac] = true;
	}
	else
	{
		residualArcPresent_[ac] = false;
	}
	includeArc(ac);
}

inline void ResidualGraph::includeArc(const ResidualArcCandidate& ac)
{
	if(!residualArcPresent_[ac] || !residualArcEnabled_[ac])
	{
		DEBUG_MSG("disabling residual arc: " << id(ac.first) << ", " << id(ac.second));
		Arc a = pairToResidualArc(ac);
		if(a == lemon::INVALID)
			return;
		erase(a);
	}
	else
	{
		DEBUG_MSG("enabling residual arc: " << id(ac.first) << ", " << id(ac.second));
		Arc a = pairToResidualArc(ac);
		if(a == lemon::INVALID)
		{
			a = addArc(ac.first, ac.second);
		}
		residualDistMap_[a] = residualArcCost_[ac];
		providedTokenMap_[a] = residualArcProvidesTokens_[ac];
		forbiddenTokenMap_[a] = residualArcForbidsTokens_[ac];
	}
}

inline bool ResidualGraph::getArcEnabledState(const OriginalArc& a)
{
	// use the latest cost and flow states
	ResidualArcCandidate ac = arcToPair(a);
	return residualArcEnabled_[ac];
}

inline void ResidualGraph::enableArc(const OriginalArc& a, bool state)
{
	// use the latest cost and flow states
	ResidualArcCandidate ac = arcToPair(a);
	residualArcEnabled_[ac] = state;
	includeArc(ac);
	
	if(useBackArcs_)
	{
		ac = arcToInversePair(a);
		residualArcEnabled_[ac] = state;
		includeArc(ac);
	}
}


/// configure required tokens of arcs
inline void ResidualGraph::addForbiddenToken(const OriginalArc& a, bool forward, Token token)
{
	ResidualArcCandidate ac = undirectedArcToPair(a, forward);
	residualArcForbidsTokens_[ac].insert(token);
	Arc resArc = pairToResidualArc(ac);
	if(resArc != lemon::INVALID)
		forbiddenTokenMap_[resArc].insert(token);
}

inline void ResidualGraph::removeForbiddenToken(const OriginalArc& a, bool forward, Token token)
{
	ResidualArcCandidate ac = undirectedArcToPair(a, forward);
	residualArcForbidsTokens_[ac].erase(token);
	Arc resArc = pairToResidualArc(ac);
	if(resArc != lemon::INVALID)
		forbiddenTokenMap_[resArc].erase(token);
}


/// configure provided tokens of arcs
inline void ResidualGraph::addProvidedToken(const OriginalArc& a, bool forward, Token token)
{
	ResidualArcCandidate ac = undirectedArcToPair(a, forward);
	residualArcProvidesTokens_[ac].insert(token);
	Arc resArc = pairToResidualArc(ac);
	if(resArc != lemon::INVALID)
		providedTokenMap_[resArc].insert(token);
}

inline void ResidualGraph::removeProvidedToken(const OriginalArc& a, bool forward, Token token)
{
	ResidualArcCandidate ac = undirectedArcToPair(a, forward);
	residualArcProvidesTokens_[ac].erase(token);
	Arc resArc = pairToResidualArc(ac);
	if(resArc != lemon::INVALID)
		providedTokenMap_[resArc].erase(token);
}


} // end namespace dpct

#endif