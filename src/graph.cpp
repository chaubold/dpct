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
{}

Graph::NodePtr Graph::addNode(const std::vector<double>& cellCountScoreDelta,
					double appearanceScoreDelta,
					double disappearanceScoreDelta,
					double divisionScoreDelta,
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

	if(config_.withDivision && !connectToSource)
	{
		ArcPtr arc(new Arc(&divisionNode_,
			node.get(),
			Arc::Division,
			divisionScoreDelta,
			node.get(),
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

Graph::ArcPtr Graph::addArc(NodePtr source, 
					NodePtr target, 
					Arc::Type type, 
					double scoreDelta,
					NodePtr dependsOnCellInNode,
					UserData* data)
{
	// assert(source in nodes_)
	// assert(target in nodes_)

	ArcPtr arc(new Arc(source.get(), 
		target.get(),
		type,
		scoreDelta,
		dependsOnCellInNode.get(),
		data));

	arcs_.push_back(arc);
	return arc;
}

} // namespace dpct
