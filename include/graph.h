#ifndef DPCT_GRAPH_H
#define DPCT_GRAPH_H

#include <memory>
#include <functional>

#include "node.h"
#include "arc.h"
#include "userdata.h"

namespace dpct
{

class Graph
{
public:
	typedef std::shared_ptr<Node> NodePtr;
	typedef std::shared_ptr<Arc>  ArcPtr;
	typedef std::vector<NodePtr> NodeVector;
	typedef std::vector<NodeVector> NodeVectorVector;
	typedef std::vector<ArcPtr> ArcVector;

	typedef std::function<void(Node*)> VisitorFunction;

	class Configuration
	{
	public:
		Configuration(bool enableAppearance, 
					bool enableDisappearance, 
                    bool enableDivision);

 		bool withAppearance;
		bool withDisappearance;
		bool withDivision;
	};

public:
	// constructor
	Graph() = delete;
	Graph(const Graph&) = delete;
	Graph(const Configuration& config);

	// Add a new node to the graph, expecting timesteps to begin with 0!
	// Appearance, disappearance and division arcs are inserted automatically.
    NodePtr addNode(size_t timestep,
                    const std::vector<double>& cellCountScoreDelta,
                    double appearanceScoreDelta = 0.0,
                    double disappearanceScoreDelta = 0.0,
                    bool connectToSource = false,
                    bool connectToSink = false,
                    UserDataPtr data = UserDataPtr());

	// add an arc between used defined nodes. Nodes have to be created first! 
	ArcPtr addMoveArc(NodePtr source, 
                    NodePtr target,
                    double scoreDelta,
                    UserDataPtr data = UserDataPtr());

	// mitosis happens along a move arc - or better two move arcs from a common
	// parent. Needs to be explicitly added
	ArcPtr allowMitosis(NodePtr parent,
						NodePtr child,
						double divisionScoreDelta);

	// get number of arcs and user defined nodes
	size_t getNumArcs() const { return arcs_.size(); }
	size_t getNumNodes() const { return numNodes_; }
	size_t getNumTimesteps() const { return nodesPerTimestep_.size(); }
	const Configuration getConfig() const { return config_; }

	// access for tracking algorithms
	Node& getSourceNode() { return sourceNode_; }
	Node& getSinkNode() { return sinkNode_; }
	void visitNodesInTimestep(size_t timestep, VisitorFunction func);
    void visitSpecialNodes(VisitorFunction func);

	void reset();

protected:
	Configuration config_;

	Node sourceNode_;
	Node sinkNode_;
	Node appearanceNode_;
	Node disappearanceNode_;
	Node divisionNode_;

	// hold a reference to all nodes such that they only get deleted when the graph
	// is destructed
	NodeVectorVector nodesPerTimestep_;
	ArcVector arcs_;
	size_t numNodes_;
};

} // namespace dpct

#endif // DPCT_GRAPH_H
