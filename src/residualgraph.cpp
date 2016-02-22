#include "residualgraph.h"
#include <limits>
#include <fstream>
#include <set>
#include "log.h"

namespace dpct
{

ResidualGraph::ResidualGraph(
		const Graph& original, 
		const std::map<OriginalNode, size_t>& nodeTimestepMap, 
		bool useBackArcs,
		bool useOrderedNodeListInBF
):
	residualDistMap_(*this),
	forbiddenTokenMap_(*this),
	providedTokenMap_(*this),
	nodeUpdateOrderMap_(*this),
	originalGraph_(original),
	useBackArcs_(useBackArcs),
	bfPredMap_(*this),
	bfDistMap_(*this),
	useOrderedNodeListInBF_(useOrderedNodeListInBF)
{
	for(Graph::NodeIt origNode(original); origNode != lemon::INVALID; ++origNode)
	{
		Node n = addNode();
		originMap_[n] = origNode;
		residualNodeMap_[origNode] = n;
		nodeUpdateOrderMap_.set(n, nodeTimestepMap.at(origNode));
	}
}

/// find a shortest path or a negative cost cycle, and return it with flow direction and cost
ResidualGraph::ShortestPathResult ResidualGraph::findShortestPath(
	const OriginalNode& origSource, 
	const OriginalNode& origTarget)
{
	Node source = residualNodeMap_.at(origSource);
	Node target = residualNodeMap_.at(origTarget);

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
			std::cout << "Retrying to find shortest path..." << ret.second << std::endl;
			OriginalNode n = originalGraph_.nodeFromId(ret.second);
			if(n == lemon::INVALID)
				throw std::runtime_error("Could not find original arc that violated the token specs!");

			// this should only be exactly one
			for(Graph::InArcIt a(originalGraph_, n); a != lemon::INVALID; ++a)
			{
				std::cout << "Disabling in-arc " << originalGraph_.id(originalGraph_.source(a)) << " -> " << ret.second << std::endl;
				ResidualArcCandidate ac = arcToInversePair(a);
				residualArcEnabled_[ac] = false;
				includeArc(ac);
			}

			LOG_MSG("Searching shortest path in graph with " << lemon::countNodes(*this)
			<< " nodes and " << lemon::countArcs(*this) << " arcs");
		}

	    BellmanFord bf(*this, residualDistMap_, providedTokenMap_, forbiddenTokenMap_);
	    bf.distMap(bfDistMap_);
	    bf.predMap(bfPredMap_);
	    bf.init();
	    if(useOrderedNodeListInBF_)
	    	bf.addSource(source, nodeUpdateOrderMap_);
	   	else
	   		bf.addSource(source);

		// * limiting the number of iterations to 1000 reduces the runtime of drosophila 10x (3200sec -> 380sec)!!
		// * checking for negative cycles each round brings runtime down to 82 secs
		// * checking for negative cycles every 100 iterations yields runtime of 53secs!
		// BUT: the number of paths found changes, which means we are not finding the same things... (different negative cycles?)
		bool foundPath = bf.checkedStart(300, 2000);
		if(foundPath)
	    {	
	    	DEBUG_MSG("Found path");
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
	            		LOG_MSG("Found loop in path!");
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
	        	pathCost += residualArcCost_.at(ac);
	            std::pair<Arc, bool> arcForward = pairToOriginalArc(ac);
	        	flow = arcForward.second ? 1 : -1;
	            p.push_back(std::make_pair(arcForward.first, flow));
	        }
	    }

		// analyze the new path
	    ret = pathSatisfiesTokenSpecs(p);
	    if(!ret.first)
	    {
	    	std::cout << "############## Found path that violates the token specs! " << ret.second << std::endl;
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
			<< "cost=" << residualArcCost_.at(ac) << "\" ";

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
			<< "cost=" << residualArcCost_.at(ac) 
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
