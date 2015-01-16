#ifndef DPCT_GRAPH_H
#define DPCT_GRAPH_H

#include "node.h"
#include "arc.h"
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
					double divisionScoreDelta = 0.0,
		 			bool connectToSource = false,
		 			bool connectToSink = false,
		 			UserData* data = nullptr);

	/// add an arc between used defined nodes. Nodes have to be created first! 
	ArcPtr addArc(NodePtr source, 
				NodePtr target, 
				Arc::Type type, 
				double scoreDelta = 0.0,
				NodePtr dependsOnCellInNode = nullptr,
				UserData* data = nullptr);

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
