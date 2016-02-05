#include "residualgraph.h"
#include <limits>
#include <fstream>
#include <set>
#include "log.h"

namespace dpct
{

ResidualGraph::ResidualGraph(const Graph& original, bool useBackArcs):
	residualDistMap_(*this),
	forbiddenTokenMap_(*this),
	providedTokenMap_(*this),
	originalGraph_(original),
	useBackArcs_(useBackArcs)
{
	for(Graph::NodeIt origNode(original); origNode != lemon::INVALID; ++origNode)
	{
		Node n = addNode();
		originMap_[n] = origNode;
		residualNodeMap_[origNode] = n;
	}
}

void ResidualGraph::updateArc(const OriginalArc& a, bool forward, double cost, int capacity)
{
	if(!useBackArcs_ && !forward)
		return;
	ResidualArcCandidate ac = undirectedArcToPair(a, forward);
	updateResidualArc(ac, cost, capacity);
}

void ResidualGraph::updateResidualArc(const ResidualArcCandidate& ac, double cost, int capacity)
{
	DEBUG_MSG("Updating residual arc (" << id(ac.first) << ", " << id(ac.second) << ") with cost " 
			<< cost << " and capacity " << capacity);
	if(capacity > 0)
	{
		residualArcCost_[ac] = cost;
		residualArcPresent_[ac] = true;
		enableArc(ac, true);
	}
	else
	{
		residualArcPresent_[ac] = false;
		enableArc(ac, false);
	}
}

void ResidualGraph::enableArc(const ResidualArcCandidate& ac, bool state)
{
	DEBUG_MSG((state? "enabling" : "disabling") << " residual arc: " 
		<< id(ac.first) << ", " << id(ac.second));
	if(!state || !residualArcPresent_[ac])
	{
		Arc a = pairToResidualArc(ac);
		if(a == lemon::INVALID)
			return;
		erase(a);
	}
	else
	{
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

void ResidualGraph::enableArc(const OriginalArc& a, bool state)
{
	// use the latest cost and flow states
	enableArc(arcToPair(a), state);
	
	if(useBackArcs_)
		enableArc(arcToInversePair(a), state);
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

	// TODO: reuse distMap and predMap of bf to avoid new and delete in each iteration?

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
				// enableArc(arcToInversePair(a), false);
				ResidualArcCandidate ac = arcToInversePair(a);
				Arc resArc = pairToResidualArc(ac);
				if(resArc == lemon::INVALID)
					std::cout << "Could not delete arc as it is not present in residual graph" << std::endl;
				erase(resArc);
				if(pairToResidualArc(ac) != lemon::INVALID)
					throw std::runtime_error("Deleting arc failed");
			}

			LOG_MSG("Searching shortest path in graph with " << lemon::countNodes(*this)
			<< " nodes and " << lemon::countArcs(*this) << " arcs");
		}

	    BellmanFord bf(*this, residualDistMap_, providedTokenMap_, forbiddenTokenMap_);
	    bf.init();
	    bf.addSource(source);

		// * limiting the number of iterations to 1000 reduces the runtime of drosophila 10x (3200sec -> 380sec)!!
		// * checking for negative cycles each round brings runtime down to 82 secs
		// * checking for negative cycles every 100 iterations yields runtime of 53secs!
		// BUT: the number of paths found changes, which means we are not finding the same things... (different negative cycles?)
		bool foundPath = bf.checkedStart(300, 1000);
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

	 //    // if we disabled an arc due to a token violation, reverse this decision now 
		// if(!ret.first)
		// {
		// 	std::cout << "Re-enabling in-arc of " << ret.second << std::endl;
		// 	OriginalNode n = originalGraph_.nodeFromId(ret.second);
		// 	// this should only be exactly one
		// 	for(Graph::InArcIt a(originalGraph_, n); a != lemon::INVALID; ++a)
		// 	{
		// 		if(!residualArcPresent_[arcToInversePair(a)])
		// 			std::cout << "!!!WARNING!!! Cannot re-enable arc because it is set to not present" << std::endl;
		// 		enableArc(arcToInversePair(a), true);
		// 	}
		// }

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

/// configure required tokens of arcs
void ResidualGraph::addForbiddenToken(const OriginalArc& a, bool forward, Token token)
{
	ResidualArcCandidate ac = undirectedArcToPair(a, forward);
	residualArcForbidsTokens_[ac].insert(token);
	Arc resArc = pairToResidualArc(ac);
	if(resArc != lemon::INVALID)
		forbiddenTokenMap_[resArc].insert(token);
}

void ResidualGraph::removeForbiddenToken(const OriginalArc& a, bool forward, Token token)
{
	ResidualArcCandidate ac = undirectedArcToPair(a, forward);
	residualArcForbidsTokens_[ac].erase(token);
	Arc resArc = pairToResidualArc(ac);
	if(resArc != lemon::INVALID)
		forbiddenTokenMap_[resArc].erase(token);
}


/// configure provided tokens of arcs
void ResidualGraph::addProvidedToken(const OriginalArc& a, bool forward, Token token)
{
	ResidualArcCandidate ac = undirectedArcToPair(a, forward);
	residualArcProvidesTokens_[ac].insert(token);
	Arc resArc = pairToResidualArc(ac);
	if(resArc != lemon::INVALID)
		providedTokenMap_[resArc].insert(token);
}

void ResidualGraph::removeProvidedToken(const OriginalArc& a, bool forward, Token token)
{
	ResidualArcCandidate ac = undirectedArcToPair(a, forward);
	residualArcProvidesTokens_[ac].erase(token);
	Arc resArc = pairToResidualArc(ac);
	if(resArc != lemon::INVALID)
		providedTokenMap_[resArc].erase(token);
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
