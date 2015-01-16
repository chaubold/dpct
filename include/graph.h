#ifndef DPCT_GRAPH_H
#define DPCT_GRAPH_H

#include "node.h"
#include "arc.h"
#include "userdata.h"
#include <memory>

namespace dpct
{

class Graph
{
public:
	typedef std::shared_ptr<Node> NodePtr;
	typedef std::shared_ptr<Arc>  ArcPtr;

	class Configuration
	{
	public:
		Configuration(bool enableAppearance, 
					bool enableDisappearance, 
					bool enableDivision, 
					bool enableSwap);

 		bool withAppearance;
		bool withDisappearance;
		bool withDivision;
		bool withSwap;
	};

public:
	// constructor
	Graph() = delete;
	Graph(const Configuration& config);

	// Add a new node to the graph.
	// Appearance, disappearance and division arcs are inserted automatically.
	NodePtr addNode(const std::vector<double>& cellCountScoreDelta,
					double appearanceScoreDelta = 0.0,
					double disappearanceScoreDelta = 0.0,
		 			bool connectToSource = false,
		 			bool connectToSink = false,
		 			UserData* data = nullptr);

	// add an arc between used defined nodes. Nodes have to be created first! 
	ArcPtr addMoveArc(NodePtr source, 
				NodePtr target, 
				double scoreDelta,
				UserData* data = nullptr);

	// mitosis happens along a move arc - or better two move arcs from a common
	// parent. Needs to be explicitly added
	ArcPtr allowMitosis(NodePtr parent,
						NodePtr child,
						double divisionScoreDelta);

	// get number of arcs and user defined nodes
	size_t getNumArcs() const { return arcs_.size(); }
	size_t getNumNodes() const { return nodes_.size(); }
	const Configuration getConfig() const { return config_; }

protected:
	Configuration config_;

	Node sourceNode_;
	Node sinkNode_;
	Node appearanceNode_;
	Node disappearanceNode_;
	Node divisionNode_;

	// hold a reference to all nodes such that they only get deleted when the graph
	// is destructed
	std::vector<NodePtr> nodes_;
	std::vector<ArcPtr> arcs_;
};

} // namespace dpct

#endif // DPCT_GRAPH_H
