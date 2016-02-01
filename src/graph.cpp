#include "graph.h"
#include "log.h"

#include <assert.h>
#include <iterator>
#include <map>
#include <algorithm>
#include <fstream>

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
    sinkNode_(std::vector<double>(), std::make_shared<NameData>("Sink")),
    sourceNode_(std::vector<double>(), std::make_shared<NameData>("Source"))
{
}

Graph::NodePtr Graph::addNode(size_t timestep,
					const std::vector<double>& cellCountScoreDelta,
					const std::vector<double>& appearanceScoreDelta,
					const std::vector<double>& disappearanceScoreDelta,
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

    std::vector<double> zeroCost(cellCountScoreDelta.size(), 0);

	// create appearance and division arcs if enabled and not in first time frame
	if(config_.withAppearance && !connectToSource)
	{
		ArcPtr arc(new Arc(&sourceNode_,
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
			&sinkNode_,
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
            {0.0},
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
            {0.0},
            nullptr,
            nullptr));
        arcs_.push_back(arc);
    }

	return node;
}

Graph::ArcPtr Graph::addMoveArc(NodePtr source,
                    NodePtr target,
                    const std::vector<double>& scoreDeltas,
                    UserDataPtr data)
{
	// assert(source in nodes_)
	// assert(target in nodes_)

	ArcPtr arc(new Arc(source.get(), 
		target.get(),
		Arc::Move,
		scoreDeltas,
		nullptr,
		data));

	arcs_.push_back(arc);
	return arc;
}

Graph::ArcPtr Graph::allowMitosis(Node* parent,
                        Node* child,
                        double divisionScoreDelta)
{
    ArcPtr arc(new Arc(&sourceNode_, 
        child,
        Arc::Division,
        {divisionScoreDelta},
        parent));

    arcs_.push_back(arc);
    return arc;
}


Graph::ArcPtr Graph::allowMitosis(NodePtr parent,
						NodePtr child,
						double divisionScoreDelta)
{
	ArcPtr arc(new Arc(&sourceNode_, 
		child.get(),
		Arc::Division,
		{divisionScoreDelta},
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
    func(&sinkNode_);
}

bool Graph::isSpecialNode(const Node *n) const
{
    if(n == &sourceNode_)
        return true;
    if(n == &sinkNode_)
    	return true;
    
    return false;
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

void Graph::print() const
{
	LOG_MSG("Source Node: " << &sourceNode_);
	LOG_MSG("Sink Node: " << &sinkNode_);

	for(NodeVector nv : nodesPerTimestep_)
	{
		LOG_MSG("&&&&&&&&&&&&&& Next timestep &&&&&&&&&&&&&&&\n\n");
		
		for(NodePtr np : nv)
		{
			LOG_MSG("Node " << np.get() << " is " << np->getUserData()->toString());
			LOG_MSG("and has in Arcs:");
			for(Node::ConstArcIt it = np->getInArcsBegin(); it != np->getInArcsEnd(); ++it)
			{
				LOG_MSG("\tArc from " << (*it)->getSourceNode() << " - " << std::string(((*it)->isEnabled()) ? "enabled" : "disabled"));
			}
			LOG_MSG("and out Arcs:");
			for(Node::ConstArcIt it = np->getOutArcsBegin(); it != np->getOutArcsEnd(); ++it)
			{
				LOG_MSG("\tArc to " << (*it)->getTargetNode() << " - " << std::string(((*it)->isEnabled()) ? "enabled" : "disabled"));
			}
		}
	}
}

void Graph::printToDot(const std::string& filename) const
{
	std::ofstream out_file(filename.c_str());

    if(!out_file.good())
    {
        throw std::runtime_error("Could not open file " + filename + " to save graph to");
    }

    out_file << "digraph G {\n";

    // nodes
	for(NodeVector nv : nodesPerTimestep_)
	{	
		for(NodePtr np : nv)
		{
			out_file << "\t" << (size_t)np.get() << " [ label=\"" << np->getUserData()->toString() << "\" ]; \n" << std::flush;
		}
	}

	// source and sink
	out_file << "\t" << (size_t)&sourceNode_ << " [ label=\"" << sourceNode_.getUserData()->toString() << "\" ]; \n" << std::flush;
	out_file << "\t" << (size_t)&sinkNode_ << " [ label=\"" << sinkNode_.getUserData()->toString() << "\" ]; \n" << std::flush;

	for(NodeVector nv : nodesPerTimestep_)
	{	
		for(NodePtr np : nv)
		{	
			for(Node::ConstArcIt it = np->getOutArcsBegin(); it != np->getOutArcsEnd(); ++it)
			{
				out_file << "\t" << (size_t)(*it)->getSourceNode() << " -> " << (size_t)(*it)->getTargetNode() << " [ label=\" " << (*it)->typeAsString() << " \" ];\n" << std::flush;
			}
		}
	}

	for(Node::ConstArcIt it = sourceNode_.getOutArcsBegin(); it != sourceNode_.getOutArcsEnd(); ++it)
	{
		out_file << "\t" << (size_t)(*it)->getSourceNode() << " -> " << (size_t)(*it)->getTargetNode() << " [ label=\" " << (*it)->typeAsString() << " \" ];\n" << std::flush;
	}

    out_file << "}";
}

} // namespace dpct
