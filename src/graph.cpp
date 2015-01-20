#include "graph.h"
#include <assert.h>
#include <iterator>

namespace dpct
{

Graph::Configuration::Configuration(bool enableAppearance, 
									bool enableDisappearance, 
                                    bool enableDivision):
	withAppearance(enableAppearance),
	withDisappearance(enableDisappearance),
    withDivision(enableDivision)
{}

Graph::Graph(const Graph::Configuration& config):
	config_(config),
    numNodes_(0),
    sinkNode_({}, std::make_shared<NameData>("Sink")),
    sourceNode_({}, std::make_shared<NameData>("Source")),
    appearanceNode_({}, std::make_shared<NameData>("Appearance")),
    disappearanceNode_({}, std::make_shared<NameData>("Disappearance")),
    divisionNode_({}, std::make_shared<NameData>("Division"))
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
	std::cout << "Resetting graph" << std::endl;

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

bool Graph::isSpecialNode(Node *n) const
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

} // namespace dpct
