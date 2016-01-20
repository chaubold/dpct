#include "residualgraph.h"
#include <limits>
#include <fstream>
#include "log.h"

namespace dpct
{

ResidualGraph::ResidualGraph(const Graph& original):
	residualDistMap_(*this),
	originalGraph_(original)
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
	}
}

void ResidualGraph::enableArc(const OriginalArc& a, bool state)
{
	// use the latest cost and flow states
	enableArc(arcToPair(a), state);
	enableArc(arcToInversePair(a), state);
}

/// find a shortest path or a negative cost cycle, and return it with flow direction and cost
ResidualGraph::ShortestPathResult ResidualGraph::findShortestPath(
	const OriginalNode& origSource, 
	const OriginalNode& origTarget) const
{
	Node source = residualNodeMap_.at(origSource);
	Node target = residualNodeMap_.at(origTarget);

	DEBUG_MSG("Searching shortest path in graph with " << lemon::countNodes(*this)
			<< " nodes and " << lemon::countArcs(*this) << " arcs");

	// TODO: reuse distMap and predMap of bf to avoid new and delete in each iteration?
    BellmanFord bf(*this, residualDistMap_);
    bf.init();
    bf.addSource(source);

	Path p;
	double pathCost = 0.0;
	int flow = 0;

	// * limiting the number of iterations to 1000 reduces the runtime of drosophila 10x (3200sec -> 380sec)!!
	// * checking for negative cycles each round brings runtime down to 82 secs
	// * checking for negative cycles every 100 iterations yields runtime of 53secs!
	// BUT: the number of paths found changes, which means we are not finding the same things... (different negative cycles?)
    if(bf.checkedStart(100, 1000))
    {	
    	LOG_MSG("Found path");
    	// found path
        if(bf.reached(target))
        {
        	pathCost = bf.dist(target);
        	for(Arc a = bf.predArc(target); a != lemon::INVALID; a = bf.predArc(this->source(a)))
            {
            	DEBUG_MSG("\t residual arc (" << id(this->source(a)) << ", " << id(this->target(a)) << ")");
            	std::pair<Arc, bool> arcForward = pairToOriginalArc(residualArcToPair(a));
            	flow = arcForward.second ? 1 : -1;
                p.push_back(std::make_pair(arcForward.first, flow));
            }
        }
        else
        	pathCost = std::numeric_limits<double>::infinity();
    }
    else
    {
    	LOG_MSG("Found cycle");
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

    return std::make_pair(p, pathCost);
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

    // nodes -> will be filled automatically by the edges, drops nodes without connections
	// for(Graph::NodeIt n(*this); n != lemon::INVALID; ++n)
	// {
	// 	out_file << "\t" << id(n) << " [ label=\"" << id(n) << "\" ]; \n" << std::flush;
	// }

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
