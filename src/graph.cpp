#include "graph.h"

namespace dpct
{

Graph::Configuration::Configuration(bool enableAppearance, 
									bool enableDisappearance, 
									bool enableDivision, 
									bool enableSwap):
	withAppearance(enableAppearance),
	withDisappearance(enableDisappearance),
	withDivision(enableDivision),
	withSwap(enableSwap)
{}

Graph::Graph(const Graph::Configuration& config):
	config_(config)
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

Graph::NodePtr Graph::addNode(const std::vector<double>& cellCountScoreDelta,
					double appearanceScoreDelta,
					double disappearanceScoreDelta,
		 			bool connectToSource,
		 			bool connectToSink,
		 			UserData* data)
{
	NodePtr node(new Node(cellCountScoreDelta, data));
	nodes_.push_back(node);

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

	return node;
}

Graph::ArcPtr Graph::addMoveArc(NodePtr source, 
					NodePtr target, 
					double scoreDelta,
					UserData* data)
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

} // namespace dpct
