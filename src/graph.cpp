#include "graph.h"
#include "log.h"

#include <assert.h>
#include <iterator>
#include <map>
#include <algorithm>

namespace dpct
{

Graph::Configuration::Configuration(bool enableAppearance, 
									bool enableDisappearance, 
                                    bool enableDivision):
	withAppearance(enableAppearance),
	withDisappearance(enableDisappearance),
    withDivision(enableDivision)
{}

Graph::Graph(Graph &other,
             NodeSelectionMap node_selection_map,
             ArcSelectionMap arc_selection_map):
    config_(other.config_),
    numNodes_(0),
    sinkNode_(std::vector<double>(), std::shared_ptr<NodeOriginData>(
                  new NodeOriginData( { &other.sinkNode_ } ))),
    sourceNode_(std::vector<double>(), std::shared_ptr<NodeOriginData>(
                    new NodeOriginData( { &other.sourceNode_ } ))),
    appearanceNode_(std::vector<double>(), std::shared_ptr<NodeOriginData>(
                        new NodeOriginData( { &other.appearanceNode_ } ))),
    disappearanceNode_(std::vector<double>(), std::shared_ptr<NodeOriginData>(
                           new NodeOriginData( { &other.disappearanceNode_ } ))),
    divisionNode_(std::vector<double>(), std::shared_ptr<NodeOriginData>(
                      new NodeOriginData( { &other.divisionNode_ } ))),
    isCopiedGraph_(true)
{
    // copy selected nodes, and keep a mapping
    std::map<Node*, Node*> node_map;

    for(const auto& timestep : other.nodesPerTimestep_)
    {
        nodesPerTimestep_.push_back(NodeVector());
        for(const NodePtr& other_node : timestep)
        {
            auto selection_map_it = node_selection_map.find(other_node.get());
            if(selection_map_it == node_selection_map.end() || selection_map_it->second == true)
            {
                NodePtr node(new Node(*other_node, std::shared_ptr<NodeOriginData>(
                                          new NodeOriginData( { other_node.get() } ))));
                nodesPerTimestep_.back().push_back(node);
                node_map[other_node.get()] = node.get();
                numNodes_++;
            }
        }
    }

    auto map_node = [&](Node* n) -> Node*
    {
        if(n == nullptr)
            return nullptr;
        else if(n == &other.sinkNode_)
            return &sinkNode_;
        else if(n == &other.sourceNode_)
            return &sourceNode_;
        else if(n == &other.appearanceNode_)
            return &appearanceNode_;
        else if(n == &other.disappearanceNode_)
            return &disappearanceNode_;
        else if(n == &other.divisionNode_)
            return &divisionNode_;
        else
            return node_map[n];
    };

    // copy selected arcs, use mapping from above
    for(const ArcPtr& other_arc : other.arcs_)
    {
        auto selection_map_it = arc_selection_map.find(other_arc.get());
        if(selection_map_it == arc_selection_map.end() || selection_map_it->second == true)
        {
            if(other_arc->getType() == Arc::Division)
            {
                // mother cell must be available and selected
                Node* mother = other_arc->getObservedNode();
                if(mother == nullptr)
                    continue;

                auto mother_it = node_selection_map.find(mother);
                if(mother_it != node_selection_map.end() && mother_it->second == false)
                {
                    DEBUG_MSG("Discarding division arc due to missing mother");
                    continue;
                }

                // there must be one alternative move arc selected
                size_t numMoveArcs = 0;

                for(Node::ArcIt out_arc_it = mother->getOutArcsBegin();
                    out_arc_it != mother->getOutArcsEnd();
                    ++out_arc_it)
                {
                    if((*out_arc_it)->getType() == Arc::Move)
                    {
                        auto move_arc_it = arc_selection_map.find(other_arc.get());
                        if(move_arc_it == arc_selection_map.end() || move_arc_it->second == true)
                            numMoveArcs++;
                    }
                }

                if(numMoveArcs < 2)
                {
                    DEBUG_MSG("Discarding division arc due to missing second possible child");
                    continue;
                }
            }

            // arcs might be going from/to special nodes - is handled by map_node()
            ArcPtr arc(new Arc(*other_arc, map_node, std::shared_ptr<ArcOriginData>(
                                   new ArcOriginData({ other_arc.get() } ))));
            arcs_.push_back(arc);
        }
    }
}

Graph::Graph(const Graph::Configuration& config):
	config_(config),
    numNodes_(0),
    sinkNode_(std::vector<double>(), std::make_shared<NameData>("Sink")),
    sourceNode_(std::vector<double>(), std::make_shared<NameData>("Source")),
    appearanceNode_(std::vector<double>(), std::make_shared<NameData>("Appearance")),
    disappearanceNode_(std::vector<double>(), std::make_shared<NameData>("Disappearance")),
    divisionNode_(std::vector<double>(), std::make_shared<NameData>("Division")),
    isCopiedGraph_(false)
{
    connectSpecialNodes();
}

void Graph::connectSpecialNodes()
{
    // connect the "special" nodes to source and sink
    if(config_.withAppearance)
    {
        ArcPtr sourceToAppNode(new Arc(&sourceNode_,
            &appearanceNode_,
            Arc::Dummy,
            0.0));
        arcs_.push_back(sourceToAppNode);
    }

    if(config_.withDivision)
    {
        ArcPtr sourceToDivisionNode(new Arc(&sourceNode_,
            &divisionNode_,
            Arc::Dummy,
            0.0));
        arcs_.push_back(sourceToDivisionNode);
    }

    if(config_.withDisappearance)
    {
        ArcPtr disToSinkNode(new Arc(&disappearanceNode_,
            &sinkNode_,
            Arc::Dummy,
            0.0));
        arcs_.push_back(disToSinkNode);
    }
}

Graph::NodePtr Graph::addNode(size_t timestep,
					const std::vector<double>& cellCountScoreDelta,
					double appearanceScoreDelta,
					double disappearanceScoreDelta,
		 			bool connectToSource,
		 			bool connectToSink,
                    UserDataPtr data)
{
	NodePtr node(new Node(cellCountScoreDelta, data));
	while(nodesPerTimestep_.size() <= timestep)
	{
		nodesPerTimestep_.push_back(std::vector<NodePtr>());
	}
	nodesPerTimestep_[timestep].push_back(node);
	numNodes_++;

	// create appearance and division arcs if enabled and not in first time frame
	if(config_.withAppearance && !connectToSource)
	{
		ArcPtr arc(new Arc(&appearanceNode_,
			node.get(),
			Arc::Appearance,
			appearanceScoreDelta,
			nullptr,
			nullptr));
		arcs_.push_back(arc);
	}

	// create arc to disappearance if this is not the very last frame
	if(config_.withDisappearance && !connectToSink)
	{
		ArcPtr arc(new Arc(node.get(),
			&disappearanceNode_,
			Arc::Disappearance,
			disappearanceScoreDelta,
			nullptr,
			nullptr));
		arcs_.push_back(arc);
	}

    // create arc from source if requested
    if(connectToSource)
    {
        ArcPtr arc(new Arc(&sourceNode_,
            node.get(),
            Arc::Dummy,
            0.0,
            nullptr,
            nullptr));
        arcs_.push_back(arc);
    }

    // create arc to sink if requested
    if(connectToSink)
    {
        ArcPtr arc(new Arc(node.get(),
            &sinkNode_,
            Arc::Dummy,
            0.0,
            nullptr,
            nullptr));
        arcs_.push_back(arc);
    }

	return node;
}

Graph::ArcPtr Graph::addMoveArc(NodePtr source,
                    NodePtr target,
                    double scoreDelta,
                    UserDataPtr data)
{
	// assert(source in nodes_)
	// assert(target in nodes_)

	ArcPtr arc(new Arc(source.get(), 
		target.get(),
		Arc::Move,
		scoreDelta,
		nullptr,
		data));

	arcs_.push_back(arc);
	return arc;
}

Graph::ArcPtr Graph::allowMitosis(NodePtr parent,
						NodePtr child,
						double divisionScoreDelta)
{
	ArcPtr arc(new Arc(&divisionNode_, 
		child.get(),
		Arc::Division,
		divisionScoreDelta,
		parent.get()));

	arcs_.push_back(arc);
    return arc;
}

void Graph::reset()
{
	DEBUG_MSG("Resetting graph");

	for(NodeVectorVector::iterator v = nodesPerTimestep_.begin(); v != nodesPerTimestep_.end(); ++v)
	{
		for(NodeVector::iterator it = v->begin(); it != v->end(); ++it)
		{
			it->reset();
		}
	}

	for(ArcVector::iterator it = arcs_.begin(); it != arcs_.end(); ++it)
	{
		it->reset();
	}

	sourceNode_.reset();
	sinkNode_.reset();
	appearanceNode_.reset();
	disappearanceNode_.reset();
	divisionNode_.reset();
}

void Graph::visitNodesInTimestep(size_t timestep, Graph::VisitorFunction func)
{
	assert(timestep < nodesPerTimestep_.size());

	for(NodeVector::iterator it = nodesPerTimestep_[timestep].begin(); it != nodesPerTimestep_[timestep].end(); ++it)
	{
		func(it->get());
    }
}

void Graph::visitSpecialNodes(Graph::VisitorFunction func)
{
    func(&sourceNode_);
    func(&appearanceNode_);
    func(&disappearanceNode_);
    func(&divisionNode_);
    func(&sinkNode_);
}

bool Graph::isSpecialNode(const Node *n) const
{
    if(n == &sourceNode_)
        return true;

    if(n == &appearanceNode_)
    	return true;

    if(n == &disappearanceNode_)
    	return true;

    if(n == &divisionNode_)
    	return true;

    if(n == &sinkNode_)
    	return true;
    
    return false;
}

Graph::NodeSelectionMap Graph::getEmptyNodeSelectionMap() const
{
    NodeSelectionMap map;

    for(NodeVectorVector::const_iterator v = nodesPerTimestep_.begin(); v != nodesPerTimestep_.end(); ++v)
    {
        for(NodeVector::const_iterator it = v->begin(); it != v->end(); ++it)
        {
            map[it->get()] = false;
        }
    }

    return map;
}

Graph::ArcSelectionMap Graph::getEmptyArcSelectionMap() const
{
    ArcSelectionMap map;

    for(ArcVector::const_iterator it = arcs_.begin(); it != arcs_.end(); ++it)
    {
        map[it->get()] = false;
    }

    return map;
}

void Graph::selectNode(NodeSelectionMap& node_selection_map, ArcSelectionMap& arc_selection_map, Node* n) const
{
    // skip if is special node
    if(isSpecialNode(n))
        return;

    // skip if already selected
    auto selection_map_it = node_selection_map.find(n);
    if(selection_map_it != node_selection_map.end() && selection_map_it->second == true)
        return;

    // select
    node_selection_map[n] = true;

    if(n->getUserData())
    {
        DEBUG_MSG("Selecting node: " << n->getUserData()->toString());
    }
    else
    {
        DEBUG_MSG("Selecting node");
    }

//    // activate special in arcs
//    for(Node::ConstArcIt in_arc = n->getInArcsBegin(); in_arc != n->getInArcsEnd(); ++in_arc)
//    {
//        if(isSpecialNode((*in_arc)->getSourceNode()))
//        {
//            arc_selection_map[*in_arc] = true;
//        }
//    }

//    // activate special out arcs
//    for(Node::ConstArcIt out_arc = n->getOutArcsBegin(); out_arc != n->getOutArcsEnd(); ++out_arc)
//    {
//        if(isSpecialNode((*out_arc)->getTargetNode()))
//        {
//            arc_selection_map[*out_arc] = true;
//        }
//    }
}

void Graph::selectArc(NodeSelectionMap& node_selection_map, ArcSelectionMap& arc_selection_map, Arc* a) const
{
    // skip if already selected
    auto selection_map_it = arc_selection_map.find(a);
    if(selection_map_it != arc_selection_map.end() && selection_map_it->second == true)
        return;

    // select
    arc_selection_map[a] = true;

    if(a->getUserData())
    {
        DEBUG_MSG("Selecting arc: " << a->getUserData()->toString());
    }
    else
    {
        DEBUG_MSG("Selecting arc");
    }

    // activate source and target nodes if this is not a mitosis arc
    if(a->getType() == Arc::Division)
        return;

    selectNode(node_selection_map, arc_selection_map, a->getSourceNode());
    selectNode(node_selection_map, arc_selection_map, a->getTargetNode());
}

bool Graph::removeNode(Node* n)
{
    assert(n->getNumInArcs() == 0);
    assert(n->getNumOutArcs() == 0);

    for(NodeVectorVector::iterator v = nodesPerTimestep_.begin(); v != nodesPerTimestep_.end(); ++v)
    {
        for(NodeVector::iterator it = v->begin(); it != v->end(); ++it)
        {
            if(it->get() == n)
            {
                v->erase(it);
                numNodes_--;
                return true;
            }
        }
    }
    return false;
}

bool Graph::removeArc(Arc* a)
{
    a->getSourceNode()->removeOutArc(a);
    a->getTargetNode()->removeInArc(a);

    for(ArcVector::iterator it = arcs_.begin(); it != arcs_.end(); ++it)
    {
        if(it->get() == a)
        {
            arcs_.erase(it);
            return true;
        }
    }
    return false;
}

void Graph::contractLoneArcs(bool usedArcsScoreZero)
{
    if(!isCopiedGraph_)
        throw std::runtime_error("Graph Contraction relies on OriginData in nodes and arcs, "
                                 "which is only given in a copied graph");

    std::vector<Arc*> arcsToContract;
    for(ArcPtr a : arcs_)
    {
        // only contract move arcs
        if(a->getType() == Arc::Move)
        {
            if(a->getSourceNode()->getNumOutArcs() == 1 && a->getTargetNode()->getNumInArcs() == 1)
            {
                arcsToContract.push_back(a.get());
            }
        }
    }

    LOG_MSG("Found " << arcsToContract.size() << " arcs to contract");

    for(Arc* a : arcsToContract)
    {
        // add source node origin and arc origin to target node origins
        Node* target = a->getTargetNode();
        Node* source = a->getSourceNode();
        std::shared_ptr<NodeOriginData> target_origin_data = std::static_pointer_cast<NodeOriginData>(target->getUserData());
        std::shared_ptr<NodeOriginData> source_origin_data = std::static_pointer_cast<NodeOriginData>(source->getUserData());
        target_origin_data->push_back_connector(std::static_pointer_cast<ArcOriginData>(a->getUserData())->getOriginsReverseOrder().back());
        target_origin_data->push_back(*source_origin_data);

        // update costs of target node (sum cellCountScoreDelta_'s, add arc cost to [1] or to all [i] > 1)
        target->accumulateScoreDelta(source);
        target->addArcCost(a, usedArcsScoreZero);

        // link incoming arcs to target node
        std::vector<Arc*> arcsToPointToTarget;
        for(Node::ArcIt it = source->getInArcsBegin(); it != source->getInArcsEnd(); ++it)
            arcsToPointToTarget.push_back(*it);
        for(Arc* b : arcsToPointToTarget)
            b->changeTargetTo(target);

        // remove source node and arc
        removeArc(a);
        removeNode(source);
    }
}

} // namespace dpct
