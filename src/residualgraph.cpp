#include "residualgraph.h"
#include <limits>
#include <fstream>
#include <set>
#include <chrono>
#include "log.h"

namespace dpct
{

ResidualGraph::ResidualGraph(
		const Graph& original, 
		const OriginalNode& origSource, 
		const std::map<OriginalNode, size_t>& nodeTimestepMap, 
		bool useBackArcs,
		bool useOrderedNodeListInBF
):
	originalGraph_(original),
	useBackArcs_(useBackArcs),
	useOrderedNodeListInBF_(useOrderedNodeListInBF),
	residualDistMap_(*this),
	nodeUpdateOrderMap_(*this),
	bfDistMap_(*this),
	bfPredMap_(*this),
	bf(*this, residualDistMap_, bfProcess_, bfNextProcess_),
	firstPath_(true)
{
	reserveNode(lemon::countNodes(original));
	reserveArc(2 * lemon::countArcs(original));

	for(Graph::NodeIt origNode(original); origNode != lemon::INVALID; ++origNode)
	{
		Node n = addNode();
		originMap_[n] = origNode;
		residualNodeMap_[origNode] = n;
		nodeUpdateOrderMap_.set(n, nodeTimestepMap.at(origNode));
	}

	for(Graph::ArcIt origArc(original); origArc != lemon::INVALID; ++origArc)
	{
		residualArcProvidesTokens_[arcToPair(origArc)] = {};
		residualArcProvidesTokens_[arcToInversePair(origArc)] = {};
		residualArcForbidsTokens_[arcToPair(origArc)] = {};
		residualArcForbidsTokens_[arcToInversePair(origArc)] = {};
		residualArcMap_[arcToPair(origArc)] = ResidualArcProperties();
		residualArcMap_[arcToInversePair(origArc)] = ResidualArcProperties();
	}

	bfProcess_.reserve(lemon::countNodes(*this));
	bfNextProcess_.reserve(lemon::countNodes(*this));
	dirtyNodes_.reserve(lemon::countNodes(*this));

    bf.distMap(bfDistMap_);
    bf.predMap(bfPredMap_);
    source_ = residualNodeMap_.at(origSource);
}

/// find a shortest path or a negative cost cycle, and return it with flow direction and cost
ResidualGraph::ShortestPathResult ResidualGraph::findShortestPath(
	const std::vector<OriginalNode>& origTargets,
	bool partialBFUpdates)
{
	typedef std::chrono::time_point<std::chrono::high_resolution_clock> TimePoint;
	DEBUG_MSG("Searching shortest path in graph with " << lemon::countNodes(*this)
			<< " nodes and " << lemon::countArcs(*this) << " arcs");

	Path p;
	double pathCost = 0.0;
	int flow = 0;
	std::pair<bool, Token> ret = std::make_pair(true, 0);

	do{
		// if the last path found a token violation, we'll remove the back arc of the mother for this iteration
		if(!ret.first)
		{
			DEBUG_MSG("Retrying to find shortest path..." << ret.second);
			OriginalNode n = originalGraph_.nodeFromId(ret.second);
			if(n == lemon::INVALID)
				throw std::runtime_error("Could not find original arc that violated the token specs!");

			// this should only be exactly one
			for(Graph::InArcIt a(originalGraph_, n); a != lemon::INVALID; ++a)
			{
				DEBUG_MSG("Disabling in-arc " << originalGraph_.id(originalGraph_.source(a)) << " -> " << ret.second);
				ResidualArcCandidate ac = arcToInversePair(a);
				ResidualArcProperties& arcProps = residualArcMap_[ac];
				arcProps.enabled = false;
				includeArc(ac, arcProps);
			}

			DEBUG_MSG("Searching shortest path in graph with " << lemon::countNodes(*this)
			<< " nodes and " << lemon::countArcs(*this) << " arcs");
		}

	// prepare for new iteration
	TimePoint iterationInitTime = std::chrono::high_resolution_clock::now();
		if(firstPath_ || !partialBFUpdates)
		{
			firstPath_ = false;
			bf.init();
		    if(useOrderedNodeListInBF_)
		    	bf.addSource(source_, nodeUpdateOrderMap_);
		   	else
		   		bf.addSource(source_);
		}
		else if(!dirtyNodes_.empty())
		{
			DEBUG_MSG("Running BF Update for " << dirtyNodes_.size() << " nodes");
			bf.update(dirtyNodes_, nodeUpdateOrderMap_);
		}
		dirtyNodes_.clear();
	    

		// * limiting the number of iterations to 1000 reduces the runtime of drosophila 10x (3200sec -> 380sec)!!
		// * checking for negative cycles each round brings runtime down to 82 secs
		// * checking for negative cycles every 100 iterations yields runtime of 53secs!
		// BUT: the number of paths found changes, which means we are not finding the same things... (different negative cycles?)
		TimePoint iterationStartTime = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed_seconds = iterationStartTime - iterationInitTime;
		DEBUG_MSG("initializing BF took " << elapsed_seconds.count() << " secs");
		
		bool foundPath = bf.checkedStart(300, 0);
		TimePoint iterationEndTime = std::chrono::high_resolution_clock::now();
		elapsed_seconds = iterationEndTime - iterationStartTime;
		DEBUG_MSG("BF took " << elapsed_seconds.count() << " secs");

		if(foundPath)
	    {	
	    	DEBUG_MSG("Found path");

	    	// we are fine if we reach any target, use the one with the lowest cost
	    	std::vector<double> targetDistances;
	    	for(auto ot : origTargets)
	    	{
		    	Node target = residualNodeMap_.at(ot);
		    	targetDistances.push_back(bf.dist(target));
		    }

		    size_t targetIndex = std::distance(targetDistances.begin(), std::min_element(targetDistances.begin(), targetDistances.end()));
		    Node target = residualNodeMap_.at(origTargets[targetIndex]);

	    	// found path
	        if(bf.reached(target))
	        {
	        	pathCost = bf.dist(target);
	        	for(Arc a = bf.predArc(target); a != lemon::INVALID; a = bf.predArc(this->source(a)))
	            {
	            	DEBUG_MSG("\t residual arc (" << id(this->source(a)) << ", " << id(this->target(a)) << ")");
	            	std::pair<Arc, bool> arcForward = pairToOriginalArc(residualArcToPair(a));
	            	flow = arcForward.second ? 1 : -1;
	            	if(std::find(p.begin(), p.end(), std::make_pair(arcForward.first, flow)) != p.end())
	            	{
	            		// throw std::runtime_error("Found loop in path!");
	            		DEBUG_MSG("Found loop in path!");
	            		p.push_back(std::make_pair(arcForward.first, flow));
	            		return std::make_pair(p, std::numeric_limits<double>::infinity());
	            		// p.clear();
	            		// foundPath = false;
	            		// break;
	            	}
	                p.push_back(std::make_pair(arcForward.first, flow));
	            }
	        }
	        else
	        	pathCost = std::numeric_limits<double>::infinity();
	    }
	    if(!foundPath)
	    {
	    	DEBUG_MSG("Found cycle");
	    	// found cycle
	    	lemon::Path<ResidualGraph> path = bf.negativeCycle();
	    	ResidualArcCandidate ac;
	    	for(lemon::Path<ResidualGraph>::ArcIt a(path); a != lemon::INVALID; ++a)
	        {
	        	DEBUG_MSG("\t residual arc (" << id(this->source(a)) << ", " << id(this->target(a)) << ")");
	        	ac = residualArcToPair(a);
	        	pathCost += residualArcMap_.at(ac).cost;
	            std::pair<Arc, bool> arcForward = pairToOriginalArc(ac);
	        	flow = arcForward.second ? 1 : -1;
	            p.push_back(std::make_pair(arcForward.first, flow));
	        }

	        // we need to initialize BF again from scratch, as a neg weight cycle 
	        // invalidates most of the distance map
	        firstPath_ = true;
	    }

	    TimePoint pathExtractionTime = std::chrono::high_resolution_clock::now();
		elapsed_seconds = pathExtractionTime - iterationEndTime;
		DEBUG_MSG("extracting path took " << elapsed_seconds.count() << " secs");

		// analyze the new path
	    ret = pathSatisfiesTokenSpecs(p);
	    if(!ret.first)
	    {
	    	DEBUG_MSG("############## Found path that violates the token specs! " << ret.second);
	    	p.clear();
	    	pathCost = 0.0;
	    }
	} while(!ret.first);

    return std::make_pair(p, pathCost);
}

std::pair<bool, ResidualGraph::Token> ResidualGraph::pathSatisfiesTokenSpecs(const Path& p) const
{
	TokenSet collectedTokens;
	TokenSet forbiddenTokens;
	// walk in reverse order as we build the path back to front in findShortestPath()
	for(auto af = p.rbegin(); af != p.rend(); ++af)
	{
		Arc a = af->first;
		bool forward = af->second > 0;
		ResidualArcCandidate ac = undirectedArcToPair(a, forward);
		const TokenSet& arcForbiddenTokens = residualArcForbidsTokens_.at(ac);
		const TokenSet& providedTokens = residualArcProvidesTokens_.at(ac);

		// update collected tokens
		collectedTokens.insert(providedTokens.begin(), providedTokens.end());
		forbiddenTokens.insert(arcForbiddenTokens.begin(), arcForbiddenTokens.end());
	}

	// no forbidden token is allowed in collected
	for(Token t : collectedTokens)
	{
		if(forbiddenTokens.count(t) > 0)
			return std::make_pair(false, t);
	}

	return std::make_pair(true, 0);
}

void ResidualGraph::fullGraphToDot(const std::string& filename, const Path& p) const
{
	std::ofstream out_file(filename.c_str());

    if(!out_file.good())
    {
        throw std::runtime_error("Could not open file " + filename + " to save graph to");
    }

    out_file << "digraph G {\n";

    // nodes
	for(Graph::NodeIt n(*this); n != lemon::INVALID; ++n)
	{
		out_file << "\t" << id(n) << " [ label=\"" << id(n) << "\" ]; \n" << std::flush;
	}

	// arcs
	for(Graph::ArcIt a(*this); a != lemon::INVALID; ++a)
	{
		ResidualArcCandidate ac = residualArcToPair(a);
		out_file << "\t" << id(source(a)) << " -> " << id(target(a)) << " [ label=\"" 
			<< "cost=" << residualArcMap_.at(ac).cost << "\" ";

		// highlight arcs on the given path
		for(const std::pair<OriginalArc, int>& af : p)
		{
			if(ac == arcToPair(af.first) || ac == arcToInversePair(af.first))
				out_file << "color=\"red\" fontcolor=\"red\" ";
		}

		out_file << "]; \n" << std::flush;
	}

    out_file << "}";
}

void ResidualGraph::toDot(const std::string& filename, const Path& p, Node& s, Node& t) const
{
	std::ofstream out_file(filename.c_str());

    if(!out_file.good())
    {
        throw std::runtime_error("Could not open file " + filename + " to save graph to");
    }

    out_file << "digraph G {\n";

    // find all nodes participating in path p
    std::set<Node> nodesOnPath;
    for(const std::pair<OriginalArc, int>& af : p)
	{
		ResidualArcCandidate ac = arcToPair(af.first);
		nodesOnPath.insert(ac.first);
		nodesOnPath.insert(ac.second);
	}

	// arcs
	for(Graph::ArcIt a(*this); a != lemon::INVALID; ++a)
	{
		ResidualArcCandidate ac = residualArcToPair(a);

		// only draw arcs that are close to the path
		if(nodesOnPath.count(ac.first) == 0 && nodesOnPath.count(ac.second) == 0)
			continue;
		// only draw arcs that are not connected to source or sink
		if(ac.first == s || ac.first == t || ac.first == s || ac.second == t)
			continue;

		out_file << "\t" << id(ac.first) << " -> " << id(ac.second) << " [ label=\"" 
			<< "cost=" << residualArcMap_.at(ac).cost
			<< " originSource=" << originalGraph_.id(originMap_.at(ac.first))
			<< " originTarget=" << originalGraph_.id(originMap_.at(ac.second))
			<< "\" ";

		// highlight arcs on the given path
		for(const std::pair<OriginalArc, int>& af : p)
		{
			if(ac == arcToPair(af.first) || ac == arcToInversePair(af.first))
				out_file << "color=\"red\" fontcolor=\"red\" ";
		}

		out_file << "]; \n" << std::flush;
	}

    out_file << "}";
}

} // end namespace dpct
