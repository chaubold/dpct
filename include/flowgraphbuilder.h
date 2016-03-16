#ifndef FLOW_GRAPH_BUILDER
#define FLOW_GRAPH_BUILDER

#include "graphbuilder.h"
#include "flowgraph.h"

namespace dpct
{

// ----------------------------------------------------------------------------------------
/**
 * @brief Implementation of building methods for the flow graph
 * 
 */
class FlowGraphBuilder : public GraphBuilder {
public:
	FlowGraphBuilder(FlowGraph* graph):
		graph_(graph)
	{}

	void addNode(
		size_t id,
		const CostDeltaVector& detectionCosts,
		const CostDeltaVector& detectionCostDeltas, 
		const CostDeltaVector& appearanceCostDeltas, 
		const CostDeltaVector& disappearanceCostDeltas,
		size_t targetIdx=0)
	{
		FlowGraph::FullNode n = graph_->addNode(detectionCostDeltas, idToTimestepsMap_[id].second);
		idToFlowGraphNodeMap_[id] = n;

		if(appearanceCostDeltas.size() > 0)
			graph_->addArc(graph_->getSource(), n.u, appearanceCostDeltas);

		if(disappearanceCostDeltas.size() > 0)
			graph_->addArc(n.v, graph_->getTarget(targetIdx), disappearanceCostDeltas);
	}

	void addArc(size_t srcId, size_t destId, const CostDeltaVector& costDeltas)
	{
		FlowGraph::Arc a = graph_->addArc(idToFlowGraphNodeMap_[srcId], idToFlowGraphNodeMap_[destId], costDeltas);
		idTupleToFlowGraphArcMap_[std::make_pair(srcId, destId)] = a;
	}

	void allowMitosis(size_t id, ValueType divisionCostDelta)
	{
		FlowGraph::Arc a = graph_->allowMitosis(idToFlowGraphNodeMap_[id], divisionCostDelta);
		idToFlowGraphDivisionArcMap_[id] = a;
	}

	NodeValueMap getNodeValues()
	{
		NodeValueMap nodeValueMap;
		const FlowGraph::FlowMap& flowMap = graph_->getFlowMap();

		for(auto iter : idToFlowGraphNodeMap_)
		{
			nodeValueMap[iter.first] = flowMap[iter.second.a];
		}

		return nodeValueMap;
	}

	ArcValueMap getArcValues()
	{
		ArcValueMap arcValueMap;
		const FlowGraph::FlowMap& flowMap = graph_->getFlowMap();

		for(auto iter : idTupleToFlowGraphArcMap_)
		{
			arcValueMap[iter.first] = flowMap[iter.second];
		}

		return arcValueMap;
	}

	DivisionValueMap getDivisionValues()
	{
		DivisionValueMap divisionValueMap;
		const FlowGraph::FlowMap& flowMap = graph_->getFlowMap();

		for(auto iter : idToFlowGraphDivisionArcMap_)
		{
			divisionValueMap[iter.first] = flowMap[iter.second] == 1;
		}

		return divisionValueMap;
	}

	FlowGraph::Arc getAppearanceArc(size_t nodeId)
	{
		for(FlowGraph::Graph::InArcIt ia(graph_->getGraph(), idToFlowGraphNodeMap_[nodeId].u); ia != lemon::INVALID; ++ia)
		{
			if(graph_->getGraph().source(ia) == graph_->getSource())
			{
				return ia;
			}
		}
		throw std::runtime_error("could not find appearance arc");
	}

	FlowGraph::Arc getDisappearanceArc(size_t nodeId)
	{
		for(FlowGraph::Graph::OutArcIt oa(graph_->getGraph(), idToFlowGraphNodeMap_[nodeId].v); oa != lemon::INVALID; ++oa)
		{
			if(graph_->isTarget(graph_->getGraph().target(oa)))
			{
				return oa;
			}
		}
		throw std::runtime_error("could not find disappearance arc");
	}

	FlowGraph::Arc getNodeArc(size_t nodeId)
	{
		return idToFlowGraphNodeMap_[nodeId].a;
	}

	FlowGraph::Arc getMoveArc(std::pair<size_t, size_t> ids)
	{
		return idTupleToFlowGraphArcMap_[ids];
	}

	std::pair<FlowGraph::Arc, FlowGraph::Arc> getDivisionArcs(size_t parent, size_t child)
	{
		for(FlowGraph::Graph::OutArcIt oa(graph_->getGraph(), graph_->getGraph().target(idToFlowGraphDivisionArcMap_[parent])); 
			oa != lemon::INVALID; ++oa)
		{
			if(graph_->getGraph().target(oa) == idToFlowGraphNodeMap_[child].u)
			{
				return std::make_pair(idToFlowGraphDivisionArcMap_[parent], oa);
			}
		}
		throw std::runtime_error("could not find division arc");
	}

private:
	/// pointer to original flow graph
	FlowGraph* graph_;

	/// mapping from id to flowgraph nodes
	std::map<size_t, FlowGraph::FullNode> idToFlowGraphNodeMap_;

	/// mapping from id to flowgraph division arc
	std::map<size_t, FlowGraph::Arc> idToFlowGraphDivisionArcMap_;

	/// mapping from tuple (id,id) to flowgraph arc
	std::map<std::pair<size_t, size_t>, FlowGraph::Arc> idTupleToFlowGraphArcMap_;
};

}

#endif