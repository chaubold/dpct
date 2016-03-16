#ifndef JSON_GRAPH_READER
#define JSON_GRAPH_READER 

#include <json/json.h>

#include "trackingalgorithm.h"
#include "log.h"
#include "graphbuilder.h"

namespace dpct
{

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
		DisappearanceTarget,
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
