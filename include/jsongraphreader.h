#ifndef JSON_GRAPH_READER
#define JSON_GRAPH_READER 

#include <json/json.h>

#include "flowgraph.h"
#include "graph.h"
#include "trackingalgorithm.h"
#include "log.h"

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
		const CostDeltaVector& detectionCosts,
		const CostDeltaVector& detectionCostDeltas, 
		const CostDeltaVector& appearanceCostDeltas, 
		const CostDeltaVector& disappearanceCostDeltas) = 0;

	/**
	 * @brief Specify in which timestep a node lies. Must be called before adding the respective node
	 */
	void setNodeTimesteps(size_t id, std::pair<size_t, size_t> timesteps)
	{
		idToTimestepsMap_[id] = timesteps;
	}

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

protected:
	/// mapping from id to timesteps
	std::map<size_t, std::pair<size_t, size_t>> idToTimestepsMap_;
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
		const CostDeltaVector& detectionCosts,
		const CostDeltaVector& detectionCostDeltas, 
		const CostDeltaVector& appearanceCostDeltas, 
		const CostDeltaVector& disappearanceCostDeltas)
	{
		FlowGraph::FullNode n = graph_->addNode(detectionCostDeltas, idToTimestepsMap_[id].second);
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

	/**
	 * @brief Magnusson performs score maximization, so we have to multiply our cost deltas by -1
	 */
	CostDeltaVector flipSign(const CostDeltaVector& vec)
	{
		CostDeltaVector newVec;
		for(auto a : vec)
			newVec.push_back(-1.0 * a);
		return newVec;
	}

	void addNode(
		size_t id,
		const CostDeltaVector& detectionCosts,
		const CostDeltaVector& detectionCostDeltas, 
		const CostDeltaVector& appearanceCostDeltas, 
		const CostDeltaVector& disappearanceCostDeltas)
	{
		if(idToTimestepsMap_.find(id) == idToTimestepsMap_.end())
			throw std::runtime_error("Node timesteps must be set for Magnusson to work");
		
		size_t timestep = idToTimestepsMap_[id].second;
		Graph::NodePtr n = graph_->addNode(timestep, flipSign(detectionCosts), flipSign(appearanceCostDeltas), flipSign(disappearanceCostDeltas), false, false);
		idToGraphNodeMap_[id] = n;
	}

	void addArc(size_t srcId, size_t destId, const CostDeltaVector& costDeltas)
	{
		if(idToGraphNodeMap_.find(srcId) == idToGraphNodeMap_.end())
			throw std::runtime_error("Trying to add link but source node is not present in map");
		if(idToGraphNodeMap_.find(destId) == idToGraphNodeMap_.end())
			throw std::runtime_error("Trying to add link but destination node is not present in map");
		Graph::ArcPtr a = graph_->addMoveArc(idToGraphNodeMap_[srcId], idToGraphNodeMap_[destId], flipSign(costDeltas));
		idTupleToGraphArcMap_[std::make_pair(srcId, destId)] = a;
	}

	void allowMitosis(size_t id, ValueType divisionCostDelta)
	{
		if(idToGraphNodeMap_.find(id) == idToGraphNodeMap_.end())
			throw std::runtime_error("Trying to add division but parent node is not present in map");
		Graph::NodePtr n = idToGraphNodeMap_[id];
		bool foundAny = false;
		n->visitOutArcs([&](Arc* a){
			if(a->getTargetNode() == nullptr)
				throw std::runtime_error("Found out arc pointing to nullptr!!!");
			if(a != n->getDisappearanceArc() && !graph_->isSpecialNode(a->getTargetNode()))
			{
				Graph::ArcPtr ap = graph_->allowMitosis(n.get(), a->getTargetNode(), -1.0 * divisionCostDelta);
				divisionArcs_.push_back(ap);
				foundAny = true;
			}
		});

		if(foundAny)
			idToGraphDivisionMap_[id] = n;
		else
			DEBUG_MSG("Could not add division arcs for node " << id);
	}

	NodeValueMap getNodeValues()
	{
		if(magnussonNodeValueMap_.size() == 0)
			throw std::runtime_error("No active nodes were found when exporting results!");

		NodeValueMap nodeValueMap;
		
		for(auto iter : idToGraphNodeMap_)
		{
			if(magnussonNodeValueMap_.find(iter.second.get()) != magnussonNodeValueMap_.end())
				nodeValueMap[iter.first] = magnussonNodeValueMap_[iter.second.get()];
		}

		return nodeValueMap;
	}

	ArcValueMap getArcValues()
	{
		ArcValueMap arcValueMap;
		
		for(auto iter : idTupleToGraphArcMap_)
		{
			if(magnussonArcValueMap_.find(iter.second.get()) != magnussonArcValueMap_.end())
				arcValueMap[iter.first] = magnussonArcValueMap_[iter.second.get()];
		}

		return arcValueMap;
	}

	DivisionValueMap getDivisionValues()
	{
		DivisionValueMap divisionValueMap;
		
		for(auto iter : idToGraphDivisionMap_)
		{
			if(magnussonDivisionValueMap_.find(iter.second.get()) != magnussonDivisionValueMap_.end())
				divisionValueMap[iter.first] = magnussonDivisionValueMap_[iter.second.get()];
		}

		return divisionValueMap;
	}

	void getSolutionFromPaths(const std::vector<TrackingAlgorithm::Path>& paths)
	{
		if(paths.size() == 0)
		{
			LOG_MSG("Got empty paths list, not extracting solution....");
			return;
		}

		magnussonNodeValueMap_.clear();
		magnussonArcValueMap_.clear();
		magnussonDivisionValueMap_.clear();

		auto increase_object_count = [&](const Node* n){
			if(magnussonNodeValueMap_.find(n) == magnussonNodeValueMap_.end())
				magnussonNodeValueMap_[n] = 1;
			else
				magnussonNodeValueMap_[n] += 1;
		};

		auto activate_arc = [&](const Arc* a)
		{
			if(magnussonArcValueMap_.find(a) == magnussonArcValueMap_.end())
				magnussonArcValueMap_[a] = 1;
			else
				magnussonArcValueMap_[a] += 1;
		};

		// fill solution vectors from the given paths
		// for each path, increment the number of cells the nodes and arcs along the path
	    for(const TrackingAlgorithm::Path& p : paths)
	    {
	        // a path starts at the dummy-source and goes to the dummy-sink. these arcs are of type dummy, and thus skipped
	        bool first_arc_on_path = true;
	        for(const Arc* a : p)
	        {
	            assert(a != nullptr);
	            assert(a->getSourceNode() != nullptr);
	            assert(a->getTargetNode() != nullptr);

	            switch(a->getType())
	            {
	                case Arc::Move:
	                {
	                    // send one cell through the nodes
	                    if(first_arc_on_path)
	                    {
	                        increase_object_count(a->getSourceNode());
	                        first_arc_on_path = false;
	                    }
	                    increase_object_count(a->getTargetNode());

	                    // set arc to active
	                    activate_arc(a);
	                }
	                break;
	                case Arc::Appearance:
	                {
	                    // the node that appeared is set active here, so detections without further path are active as well
	                    increase_object_count(a->getTargetNode());
	                    first_arc_on_path = false;
	                }
	                break;
	                case Arc::Disappearance:
	                {
	                    if(first_arc_on_path)
	                    {
	                        increase_object_count(a->getSourceNode());
	                    }
	                    first_arc_on_path = false;
	                    // nothing to do, last node on path was already set active by previous move or appearance
	                }
	                break;
	                case Arc::Division:
	                {
	                	assert(a->getObservedNode() != nullptr);
	                	magnussonDivisionValueMap_[a->getObservedNode()] = true;

	                	// activate corresponding arc, which is not active in dpct!
	                    for(Node::ConstArcIt ai = a->getObservedNode()->getOutArcsBegin();
	                    	ai != a->getObservedNode()->getOutArcsEnd();
	                    	++ai)
	                    {
	                        if((*ai)->getTargetNode() == a->getTargetNode())
	                        {
	                            magnussonArcValueMap_[*ai] = 1;
	                        }
	                    }
	                    
	                    increase_object_count(a->getTargetNode());
	                    first_arc_on_path = false;
	                }
	                break;
	                case Arc::Swap:
	                {
	                    throw std::runtime_error("Got a swap arc even though it should have been cleaned up!");
	                }
	                break;
	                case Arc::Dummy:
	                {
	                    // do nothing
	                } break;
	                default:
	                {
	                    throw std::runtime_error("Unkown arc type");
	                }
	                break;
	            } // switch
	        } // for all arcs
	    } // for all paths
	}

private:
	/// pointer to magnusson's graph
	Graph* graph_;

	/// mapping from id to nodes
	std::map<size_t, Graph::NodePtr> idToGraphNodeMap_;

	/// mapping from id to division arc
	std::map<size_t, Graph::NodePtr> idToGraphDivisionMap_;

	/// mapping from tuple (id,id) to arc
	std::map<std::pair<size_t, size_t>, Graph::ArcPtr> idTupleToGraphArcMap_;

	/// list of all division arcs - shared ptr references so that the arc doesn't get deleted
	std::vector<Graph::ArcPtr> divisionArcs_;

	/// result maps that are filled when a set of paths is passed in
	std::map<const Node*, size_t> magnussonNodeValueMap_;
	std::map<const Arc*, size_t> magnussonArcValueMap_;
	std::map<const Node*, size_t> magnussonDivisionValueMap_;
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
