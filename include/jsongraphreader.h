#ifndef JSON_GRAPH_READER
#define JSON_GRAPH_READER 

#include <json/json.h>

#include "flowgraph.h"
#include "graph.h"
#include "trackingalgorithm.h"

namespace dpct
{

// ----------------------------------------------------------------------------------------
/**
 * @brief Base class for graph building factories used by the JsonGraphReader
 */
class GraphBuilder {
public:
	typedef double ValueType;
	typedef std::vector<ValueType> CostDeltaVector;
	typedef std::map<size_t, size_t> NodeValueMap;
	typedef std::map<size_t, bool> DivisionValueMap;
	typedef std::map<std::pair<size_t, size_t>, size_t> ArcValueMap;


	/**
	 * @brief add a node which can be indexed by its id. Costs 
	 */
	virtual void addNode(
		size_t id,
		const CostDeltaVector& detectionCostDeltas, 
		const CostDeltaVector& appearanceCostDeltas, 
		const CostDeltaVector& disappearanceCostDeltas) = 0;

	/**
	 * @brief Specify in which timestep a node lies. Must be called before adding the respective node
	 */
	virtual void setNodeTimesteps(size_t id, std::pair<size_t, size_t> timesteps){}

	/**
	 * @brief add a move arc between the given nodes with the specified cost deltas
	 */
	virtual void addArc(size_t srcId, size_t destId, const CostDeltaVector& costDeltas) = 0;

	/**
	 * @brief allow a node to divide with the specified cost change
	 */
	virtual void allowMitosis(size_t id, ValueType divisionCostDelta) = 0;

	/**
	 * @brief return a mapping from node id to value in solution
	 * @details doesn't necessarily contain all nodes, implicitly assumes that all missing ones have value zero
	 * @return copy/move constructed node value map
	 */
	virtual NodeValueMap getNodeValues() = 0;

	/**
	 * @brief return a mapping from (src node id, target node id) to value in solution
	 * @details doesn't necessarily contain all links, implicitly assumes that all missing ones have value zero
	 * @return copy/move constructed link value map
	 */
	virtual ArcValueMap getArcValues() = 0;

	/**
	 * @brief return a mapping from node id to division value in solution
	 * @details doesn't necessarily contain all nodes, implicitly assumes that all missing ones are not dividing
	 * @return copy/move constructed division value map
	 */
	virtual DivisionValueMap getDivisionValues() = 0;
};

// ----------------------------------------------------------------------------------------
/**
 * @brief Implementation of building methods for the flow graph
 * @details [long description]
 * 
 */
class FlowGraphBuilder : public GraphBuilder {
public:
	FlowGraphBuilder(FlowGraph* graph):
		graph_(graph)
	{}

	void addNode(
		size_t id,
		const CostDeltaVector& detectionCostDeltas, 
		const CostDeltaVector& appearanceCostDeltas, 
		const CostDeltaVector& disappearanceCostDeltas)
	{
		FlowGraph::FullNode n = graph_->addNode(detectionCostDeltas);
		idToFlowGraphNodeMap_[id] = n;

		if(appearanceCostDeltas.size() > 0)
			graph_->addArc(graph_->getSource(), n.u, appearanceCostDeltas);

		if(disappearanceCostDeltas.size() > 0)
			graph_->addArc(n.v, graph_->getTarget(), disappearanceCostDeltas);
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

// ----------------------------------------------------------------------------------------
/**
 * @brief Implementation of building methods for magnusson's graph
 */
class MagnussonGraphBuilder : public GraphBuilder {
public:
	MagnussonGraphBuilder(Graph* graph):
		graph_(graph)
	{}

	void setNodeTimesteps(size_t id, std::pair<size_t, size_t> timesteps)
	{
		idToTimestepsMap_[id] = timesteps;
	}

	void addNode(
		size_t id,
		const CostDeltaVector& detectionCostDeltas, 
		const CostDeltaVector& appearanceCostDeltas, 
		const CostDeltaVector& disappearanceCostDeltas)
	{
		if(idToTimestepsMap_.find(id) == idToTimestepsMap_.end())
			throw std::runtime_error("Node timesteps must be set for Magnusson to work");
		
		size_t timestep = idToTimestepsMap_[id].second;
		// TODO: invert sign of cost deltas (scores increase, costs decrease!)
		Graph::NodePtr n = graph_->addNode(timestep, detectionCostDeltas, appearanceCostDeltas[0], disappearanceCostDeltas[0], false, false);
		idToGraphNodeMap_[id] = n;
	}

	void addArc(size_t srcId, size_t destId, const CostDeltaVector& costDeltas)
	{
		Graph::ArcPtr a = graph_->addMoveArc(idToGraphNodeMap_[srcId], idToGraphNodeMap_[destId], costDeltas[0]);
		idTupleToGraphArcMap_[std::make_pair(srcId, destId)] = a;
	}

	void allowMitosis(size_t id, ValueType divisionCostDelta)
	{
		Graph::NodePtr n = idToGraphNodeMap_[id];
		n->visitOutArcs([&](Arc* a){
			if(a != n->getDisappearanceArc())
			{
				Graph::ArcPtr ap = graph_->allowMitosis(idToGraphNodeMap_[id], ap->getTargetNode()->getSharedPtr(), divisionCostDelta);
				idToGraphDivisionArcMap_[id] = ap;
			}
		});
	}

	NodeValueMap getNodeValues()
	{
		NodeValueMap nodeValueMap;
		// const Graph::FlowMap& flowMap = graph_->getFlowMap();

		for(auto iter : idToGraphNodeMap_)
		{
			// nodeValueMap[iter.first] = flowMap[iter.second.a];
		}

		return nodeValueMap;
	}

	ArcValueMap getArcValues()
	{
		ArcValueMap arcValueMap;
		// const Graph::FlowMap& flowMap = graph_->getFlowMap();

		for(auto iter : idTupleToGraphArcMap_)
		{
			// arcValueMap[iter.first] = flowMap[iter.second];
		}

		return arcValueMap;
	}

	DivisionValueMap getDivisionValues()
	{
		DivisionValueMap divisionValueMap;
		// const Graph::FlowMap& flowMap = graph_->getFlowMap();

		for(auto iter : idToGraphDivisionArcMap_)
		{
			// divisionValueMap[iter.first] = flowMap[iter.second] == 1;
		}

		return divisionValueMap;
	}

	void getSolutionFromPaths(const std::vector<TrackingAlgorithm::Path>& paths)
	{
		// somehow fill solution vectors from this
	}

private:
	/// pointer to magnusson's graph
	Graph* graph_;

	/// mapping from id to timesteps
	std::map<size_t, std::pair<size_t, size_t>> idToTimestepsMap_;

	/// mapping from id to nodes
	std::map<size_t, Graph::NodePtr> idToGraphNodeMap_;

	/// mapping from id to division arc
	std::map<size_t, Graph::ArcPtr> idToGraphDivisionArcMap_;

	/// mapping from tuple (id,id) to arc
	std::map<std::pair<size_t, size_t>, Graph::ArcPtr> idTupleToGraphArcMap_;
};


// ----------------------------------------------------------------------------------------
/**
 * @brief A json graph reader provides functions to read a flow graph from json, 
 * and stores a mapping from JSON ids to graph nodes
 * 
 */
class JsonGraphReader
{
public:
	typedef double ValueType;
	typedef std::vector<ValueType> FeatureVector;
	typedef std::vector<FeatureVector> StateFeatureVector;

private:
	/// Enumerate the strings for attributes used in the Json files
	enum class JsonTypes {Segmentations, 
		Links, 
		Exclusions, 
		LinkResults, 
		DivisionResults,
		DetectionResults,
		SrcId, 
		DestId, 
		Value, 
		Id, 
		Timestep,
		Features, 
		DivisionFeatures,
		AppearanceFeatures,
		DisappearanceFeatures,
		Weights,
		// settings-related
		Settings,
		StatesShareWeights,
		OptimizerEpGap,
		OptimizerVerbose,
		OptimizerNumThreads,
		AllowPartialMergerAppearance,
		RequireSeparateChildrenOfDivision
	};

	/// mapping from JsonTypes to strings which are used in the Json files
	static std::map<JsonTypes, std::string> JsonTypeNames;

public:
	/**
	 * @brief Construct a json graph reader that reads from the specified files
	 * @param modelFilename filename of JSON model description
	 * @param weightsFilename filename where the weights are stored as JSON
	 */
	JsonGraphReader(const std::string& modelFilename, const std::string& weightsFilename, GraphBuilder* graphBuilder);

	/**
	 * @brief Add nodes and arcs to the graph builder according to the model file. 
	 * Costs are computed from features times weights
	 */
	void createGraphFromJson();

	/**
	 * @brief Save a resulting flow map back to JSON, the flow is extracted by the graph builder
	 * 
	 * @param filename filename where the results will be stored as Json
	 */
	void saveResultJson(const std::string& filename);

	/**
	 * @return the energy of the initial state of flow based solving: no objects tracked at all
	 */
	const double getInitialStateEnergy() const { return initialStateEnergy_; }

private:
	StateFeatureVector extractFeatures(const Json::Value& entry, JsonTypes type);
	FeatureVector weightedSumOfFeatures(const StateFeatureVector& stateFeatures, const FeatureVector& weights, size_t offset, bool statesShareWeights);
	FeatureVector readWeightsFromJson(const std::string& filename);
	FeatureVector costsToScoreDeltas(const FeatureVector& costs);
	ValueType costsToScoreDelta(const FeatureVector& costs);
	size_t getNumWeights(const Json::Value& jsonHyp, JsonTypes type, bool statesShareWeights);

private:
	/// filename where the model is stored in json
	std::string modelFilename_;

	/// filename where the wheights are stored in json
	std::string weightsFilename_;

	/// the weight vector loaded from file
	FeatureVector weights_;

	/// store the model's energy of the initial solution = all zeros
	double initialStateEnergy_;

	/// the graph builder instance which provides the graph specific node/arc 
	/// creation methods as well as result parsing
	GraphBuilder* graphBuilder_;
};

} // end namespace dpct

#endif // JSON_GRAPH_READER
