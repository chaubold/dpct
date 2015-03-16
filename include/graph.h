#ifndef DPCT_GRAPH_H
#define DPCT_GRAPH_H

#include <memory>
#include <functional>
#include <map>

#include "node.h"
#include "arc.h"
#include "userdata.h"

namespace dpct
{

template<typename T>
class OriginData : public UserData
{
public:
    OriginData(const std::vector<T>& origin):
        origin_(origin)
    {}

    virtual std::string toString() const { return "OriginData"; }
    const std::vector<T>& getOrigin() const { return origin_; }

private:
    std::string name_;
    std::vector<T> origin_;
};

typedef OriginData<const Node*> NodeOriginData;
typedef OriginData<const Arc*>  ArcOriginData;

class Graph
{
public:
	typedef std::shared_ptr<Node> NodePtr;
	typedef std::shared_ptr<Arc>  ArcPtr;
	typedef std::vector<NodePtr> NodeVector;
	typedef std::vector<NodeVector> NodeVectorVector;
	typedef std::vector<ArcPtr> ArcVector;
    typedef std::map<Node*, bool> NodeSelectionMap;
    typedef std::map<Arc*, bool>  ArcSelectionMap;

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
    // constructors:
	Graph() = delete;

    // selective copy constructor, copies "special" and selected nodes.
    // if maps are empty, copies everything
    Graph(const Graph& other,
          NodeSelectionMap node_selection_map = std::map<Node*, bool>(),
          ArcSelectionMap arc_selection_map = std::map<Arc*, bool>());

    // create with config
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
    bool isSpecialNode(Node *n) const;

	void reset();

    // selection map creation. Use in conjunction with selective copy constructor
    NodeSelectionMap getEmptyNodeSelectionMap() const;
    ArcSelectionMap getEmptyArcSelectionMap() const;
    // select node and its "special arcs"
    void selectNode(NodeSelectionMap& node_selection_map,
                    ArcSelectionMap& arc_selection_map,
                    Node* n) const;
    // select arc, and source and target node if they are not active yet
    void selectArc(NodeSelectionMap& node_selection_map,
                   ArcSelectionMap& arc_selection_map,
                   Arc* a) const;

protected:
    void connectSpecialNodes();

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
