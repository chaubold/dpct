#ifndef DPCT_LEMON_GRAPH_H
#define DPCT_LEMON_GRAPH_H

#include <map>
#include <lemon/list_graph.h>

namespace dpct
{

// forward declarations
class Graph;
class Arc;
class Node;

class LemonGraph
{
public:
	typedef lemon::ListDigraph LemonDigraph;
	typedef LemonDigraph::Node LemonNode;
	typedef LemonDigraph::Arc LemonArc;
	typedef LemonDigraph::ArcMap<double> LemonArcMap;
	typedef std::map<Arc*, LemonArc> ArcToLemonArcMap;
	typedef std::map<Node*, LemonNode> NodeToLemonNodeMap;

public:
	// create from a graph
	LemonGraph(Graph* g);

	// read the scores of arcs and nodes from the original graph
	void updateArcScores();

	size_t getNumArcs()  const { return lemon::countArcs(lemonGraph_); }
	size_t getNumNodes() const { return lemon::countNodes(lemonGraph_); }

private:
	LemonDigraph lemonGraph_;
	LemonArcMap arcScoreMap_;
	ArcToLemonArcMap arcToLemonArcMap_;
	NodeToLemonNodeMap nodeToLemonNodeMap_;
	Graph* graph_;
};

} // end namespace dpct

#endif //DPCT_LEMON_GRAPH_H
