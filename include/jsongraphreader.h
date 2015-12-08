#ifndef JSON_GRAPH_READER
#define JSON_GRAPH_READER 

#include <json/json.h>

#include "flowgraph.h"

namespace dpct
{

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
	JsonGraphReader(const std::string& modelFilename, const std::string& weightsFilename);

	/**
	 * @brief Fill the given flow graph with nodes and arcs according to the model file. 
	 * Costs are computed from features times weights
	 * 
	 * @param g the given flow graph
	 * @param source the source node to use as starting point for appearance arcs
	 * @param target the target node where disappearances end
	 */
	void createFlowGraphFromJson(FlowGraph& g, FlowGraph::Node source, FlowGraph::Node target);

	/**
	 * @brief Save a resulting flow map back to JSON
	 * 
	 * @param filename filename where the results will be stored as Json
	 * @param graph the graph which was used to run the tracking
	 * @param flowMap flow map containing the result of tracking
	 */
	void saveFlowMapToResultJson(const std::string& filename, FlowGraph& graph, const FlowGraph::FlowMap& flowMap);

private:
	StateFeatureVector extractFeatures(const Json::Value& entry, JsonTypes type);
	FeatureVector weightedSumOfFeatures(const StateFeatureVector& stateFeatures, const FeatureVector& weights, size_t offset, bool statesShareWeights);
	FeatureVector readWeightsFromJson(const std::string& filename);
	FeatureVector costsToScoreDeltas(const FeatureVector& costs);
	ValueType costsToScoreDelta(const FeatureVector& costs);

private:
	/// filename where the model is stored in json
	std::string modelFilename_;

	/// filename where the wheights are stored in json
	std::string weightsFilename_;

	/// the weight vector loaded from file
	FeatureVector weights_;

	/// mapping from id to flowgraph nodes
	std::map<size_t, FlowGraph::Node> idToFlowGraphNodeMap_;

	/// mapping from id to flowgraph division arc
	std::map<size_t, FlowGraph::Arc> idToFlowGraphDivisionArcMap_;

	/// mapping from tuple (id,id) to flowgraph arc
	std::map<std::pair<size_t, size_t>, FlowGraph::Arc> idTupleToFlowGraphArcMap_;
};

} // end namespace dpct

#endif // JSON_GRAPH_READER
