#include "residualgraph.h"
#include <limits>
#include <fstream>

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

void ResidualGraph::updateForwardArc(const OriginalArc& a, double cost, int capacity)
{
	ResidualArcCandidate ac = arcToPair(a);
	updateArc(ac, cost, capacity);
}

void ResidualGraph::updateBackwardArc(const OriginalArc& a, double cost, int capacity)
{
	ResidualArcCandidate ac = arcToInversePair(a);
	updateArc(ac, cost, capacity);
}

void ResidualGraph::updateArc(const ResidualArcCandidate& ac, double cost, int capacity)
{
	std::cout << "Updating residual arc (" << id(ac.first) << ", " << id(ac.second) << ") with cost " 
			<< cost << " and capacity " << capacity << std::endl;
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
	std::cout << (state? "enabling" : "disabling") << " residual arc: " 
		<< id(ac.first) << ", " << id(ac.second) << std::endl;
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
			residualDistMap_[a] = residualArcCost_[ac];
		}
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

	std::cout << "Searching shortest path in graph with " << lemon::countNodes(*this)
			<< " nodes and " << lemon::countArcs(*this) << " arcs" << std::endl;

    BellmanFord bf(*this, residualDistMap_);
    bf.init();
    bf.addSource(source);

	Path p;
	double pathCost = 0.0;
	int flow = 0;

    if(bf.checkedStart())
    {	
    	// found path
        if(bf.reached(target))
        {
        	pathCost = bf.dist(target);
        	for(Arc a = bf.predArc(target); a != lemon::INVALID; a = bf.predArc(this->source(a)))
            {
            	std::cout << "\t residual arc (" << id(this->source(a)) << ", " << id(this->target(a)) << ")" << std::endl;
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
    	// found cycle
    	lemon::Path<ResidualGraph> path = bf.negativeCycle();
    	ResidualArcCandidate ac;
    	for(lemon::Path<ResidualGraph>::ArcIt a(path); a != lemon::INVALID; ++a)
        {
        	std::cout << "\t residual arc (" << id(this->source(a)) << ", " << id(this->target(a)) << ")" << std::endl;
        	ac = residualArcToPair(a);
        	pathCost += residualArcCost_.at(ac);
            std::pair<Arc, bool> arcForward = pairToOriginalArc(ac);
        	flow = arcForward.second ? 1 : -1;
            p.push_back(std::make_pair(arcForward.first, flow));
        }
    }

    return std::make_pair(p, pathCost);
}

void ResidualGraph::toDot(const std::string& filename, const Path& p)
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
			<< "cost=" << residualArcCost_[ac] << "\" ";

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
