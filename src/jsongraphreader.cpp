#include "jsongraphreader.h"

#include <fstream>

namespace dpct
{

std::map<JsonGraphReader::JsonTypes, std::string> JsonGraphReader::JsonTypeNames = {
	{JsonTypes::Segmentations, "segmentationHypotheses"}, 
	{JsonTypes::Links, "linkingHypotheses"}, 
	{JsonTypes::Exclusions, "exclusions"},
	{JsonTypes::LinkResults, "linkingResults"},
	{JsonTypes::DivisionResults, "divisionResults"},
	{JsonTypes::DetectionResults, "detectionResults"},
	{JsonTypes::SrcId, "src"}, 
	{JsonTypes::DestId, "dest"}, 
	{JsonTypes::Value, "value"},
	{JsonTypes::Id, "id"}, 
	{JsonTypes::Features, "features"},
	{JsonTypes::DivisionFeatures, "divisionFeatures"},
	{JsonTypes::AppearanceFeatures, "appearanceFeatures"},
	{JsonTypes::DisappearanceFeatures, "disappearanceFeatures"},
	{JsonTypes::Weights, "weights"},
	{JsonTypes::StatesShareWeights, "statesShareWeights"},
	{JsonTypes::Settings, "settings"},
	{JsonTypes::OptimizerEpGap, "optimizerEpGap"},
	{JsonTypes::OptimizerVerbose, "optimizerVerbose"},
	{JsonTypes::OptimizerNumThreads, "optimizerNumThreads"},
	{JsonTypes::AllowPartialMergerAppearance, "allowPartialMergerAppearance"},
	{JsonTypes::RequireSeparateChildrenOfDivision, "requireSeparateChildrenOfDivision"}};

JsonGraphReader::JsonGraphReader(const std::string& modelFilename, const std::string& weightsFilename):
	modelFilename_(modelFilename),
	weightsFilename_(weightsFilename)
{
}

void JsonGraphReader::createFlowGraphFromJson(FlowGraph& g, FlowGraph::Node source, FlowGraph::Node target)
{
	std::ifstream input(modelFilename_.c_str());
	if(!input.good())
		throw std::runtime_error("Could not open JSON model file for reading: " + modelFilename_);

	Json::Value root;
	input >> root;

	// get flag whether states should share weights or not
	bool statesShareWeights = false;
	if(root.isMember(JsonTypeNames[JsonTypes::Settings]))
	{
		Json::Value& settings = root[JsonTypeNames[JsonTypes::Settings]];
		if(settings.isMember(JsonTypeNames[JsonTypes::StatesShareWeights]))
			statesShareWeights = settings[JsonTypeNames[JsonTypes::StatesShareWeights]].asBool();
	}

	// ------------------------------------------------------------------------------
	// get weight vector and number of weights needed for each different variable type
	FeatureVector weights = readWeightsFromJson(weightsFilename_);
	size_t numDetWeights = 0;
	size_t numDivWeights = 0;
	size_t numAppWeights = 0;
	size_t numDisWeights = 0;
	size_t numLinkWeights = 0;

	auto getNumWeights = [&](const Json::Value& jsonHyp, JsonTypes type)
	{
		size_t numWeights;
		StateFeatureVector stateFeatVec = extractFeatures(jsonHyp, type);
		
		if(statesShareWeights)
			numWeights = stateFeatVec[0].size();
		else
		{
			numWeights = 0;
			for(auto stateFeats : stateFeatVec)
				numWeights += stateFeats.size();
		}
		return numWeights;
	};

	const Json::Value segmentationHypotheses = root[JsonTypeNames[JsonTypes::Segmentations]];
	for(int i = 0; i < (int)segmentationHypotheses.size(); i++)
	{
		const Json::Value jsonHyp = segmentationHypotheses[i];
		numDetWeights = getNumWeights(jsonHyp, JsonTypes::Features);

		if(jsonHyp.isMember(JsonTypeNames[JsonTypes::DivisionFeatures]))
			numDivWeights = getNumWeights(jsonHyp, JsonTypes::DivisionFeatures);

		if(jsonHyp.isMember(JsonTypeNames[JsonTypes::AppearanceFeatures]))
			numAppWeights = getNumWeights(jsonHyp, JsonTypes::AppearanceFeatures);

		if(jsonHyp.isMember(JsonTypeNames[JsonTypes::DisappearanceFeatures]))
			numDisWeights = getNumWeights(jsonHyp, JsonTypes::DisappearanceFeatures);
	}

	const Json::Value linkingHypotheses = root[JsonTypeNames[JsonTypes::Links]];
	for(int i = 0; i < (int)linkingHypotheses.size(); i++)
	{
		const Json::Value jsonHyp = linkingHypotheses[i];
		if(jsonHyp.isMember(JsonTypeNames[JsonTypes::Features]))
				numLinkWeights = getNumWeights(jsonHyp, JsonTypes::Features);
	}

	if(weights.size() != numDetWeights + numDivWeights + numAppWeights + numDisWeights + numLinkWeights)
	{
		std::stringstream s;
		s << "Loaded weights do not meet model requirements! Got " << weights.size() << ", need " 
			<< numDetWeights + numDivWeights + numAppWeights + numDisWeights + numLinkWeights;
		throw std::runtime_error(s.str());
	}

	size_t linkWeightOffset = 0;
	size_t detWeightOffset = linkWeightOffset + numLinkWeights;
	size_t divWeightOffset = detWeightOffset + numDetWeights;
	size_t appWeightOffset = divWeightOffset + numDivWeights;
	size_t disWeightOffset = appWeightOffset + numAppWeights;

	// ------------------------------------------------------------------------------
	// read segmentation hypotheses and add to flowgraph
	std::cout << "\tcontains " << segmentationHypotheses.size() << " segmentation hypotheses" << std::endl;
	
	for(int i = 0; i < (int)segmentationHypotheses.size(); i++)
	{
		const Json::Value jsonHyp = segmentationHypotheses[i];

		if(!jsonHyp.isMember(JsonTypeNames[JsonTypes::Id]))
			throw std::runtime_error("Cannot read detection hypothesis without Id!");
		size_t id = jsonHyp[JsonTypeNames[JsonTypes::Id]].asInt();

		if(!jsonHyp.isMember(JsonTypeNames[JsonTypes::Features]))
			throw std::runtime_error("Cannot read detection hypothesis without features!");

		FlowGraph::FullNode n = g.addNode(costsToScoreDeltas(weightedSumOfFeatures(extractFeatures(jsonHyp, JsonTypes::Features), weights, detWeightOffset, statesShareWeights)));
		idToFlowGraphNodeMap_[id] = n;

		if(jsonHyp.isMember(JsonTypeNames[JsonTypes::AppearanceFeatures]))
			g.addArc(g.getSource(), n.u, costsToScoreDeltas(weightedSumOfFeatures(extractFeatures(jsonHyp, JsonTypes::AppearanceFeatures), weights, appWeightOffset, statesShareWeights)));

		if(jsonHyp.isMember(JsonTypeNames[JsonTypes::DisappearanceFeatures]))
			g.addArc(n.v, g.getTarget(), costsToScoreDeltas(weightedSumOfFeatures(extractFeatures(jsonHyp, JsonTypes::DisappearanceFeatures), weights, disWeightOffset, statesShareWeights)));
	}

	// read linking hypotheses
	std::cout << "\tcontains " << linkingHypotheses.size() << " linking hypotheses" << std::endl;
	for(int i = 0; i < (int)linkingHypotheses.size(); i++)
	{
		const Json::Value jsonHyp = linkingHypotheses[i];

		size_t srcId = jsonHyp[JsonTypeNames[JsonTypes::SrcId]].asInt();
		size_t destId = jsonHyp[JsonTypeNames[JsonTypes::DestId]].asInt();
		FlowGraph::Arc a = g.addArc(idToFlowGraphNodeMap_[srcId], 
			idToFlowGraphNodeMap_[destId], 
			costsToScoreDeltas(weightedSumOfFeatures(extractFeatures(jsonHyp, JsonTypes::Features), weights, linkWeightOffset, statesShareWeights)));
		idTupleToFlowGraphArcMap_[std::make_pair(srcId, destId)] = a;
	}

	// read divisions
	for(int i = 0; i < (int)segmentationHypotheses.size(); i++)
	{
		const Json::Value jsonHyp = segmentationHypotheses[i];
		size_t id = jsonHyp[JsonTypeNames[JsonTypes::Id]].asInt();

		if(jsonHyp.isMember(JsonTypeNames[JsonTypes::DivisionFeatures]))
		{
			FlowGraph::Arc a = g.allowMitosis(idToFlowGraphNodeMap_[id], costsToScoreDelta(weightedSumOfFeatures(extractFeatures(jsonHyp, JsonTypes::DivisionFeatures), weights, divWeightOffset, statesShareWeights)));
			idToFlowGraphDivisionArcMap_[id] = a;
		}
	}

	// read exclusion constraints between detections
	// const Json::Value exclusions = root[JsonTypeNames[JsonTypes::Exclusions]];
	// std::cout << "\tcontains " << exclusions.size() << " exclusions" << std::endl;
	// for(int i = 0; i < (int)exclusions.size(); i++)
	// {
	// 	const Json::Value jsonExc = exclusions[i];
	// 	// TODO: implement some API for those?!
	// }
	if(root.isMember(JsonTypeNames[JsonTypes::Exclusions]) && root[JsonTypeNames[JsonTypes::Exclusions]].size() > 0)
		throw std::runtime_error("FlowSolver cannot deal with exclusion constraints yet!");
}

void JsonGraphReader::saveFlowMapToResultJson(const std::string& filename, FlowGraph& graph, const FlowGraph::FlowMap& flowMap)
{
	std::ofstream output(filename.c_str());
	if(!output.good())
		throw std::runtime_error("Could not open JSON result file for saving: " + filename);

	Json::Value root;

	// save links
	Json::Value& linksJson = root[JsonTypeNames[JsonTypes::LinkResults]];
	for(auto iter : idTupleToFlowGraphArcMap_)
	{
		int value = flowMap[iter.second];
		assert(value >= 0);
		if(value > 0)
		{
			Json::Value val;
			val[JsonTypeNames[JsonTypes::SrcId]] = Json::Value((unsigned int)iter.first.first);
			val[JsonTypeNames[JsonTypes::DestId]] = Json::Value((unsigned int)iter.first.second);
			val[JsonTypeNames[JsonTypes::Value]] = Json::Value((unsigned int)value);
			linksJson.append(val);
		}
	}

	// save divisions
	Json::Value& divisionsJson = root[JsonTypeNames[JsonTypes::DivisionResults]];
	for(auto iter : idToFlowGraphDivisionArcMap_)
	{
		size_t value = flowMap[iter.second];
		if(value > 0)
		{
			Json::Value val;
			val[JsonTypeNames[JsonTypes::Id]] = Json::Value((unsigned int)iter.first);
			val[JsonTypeNames[JsonTypes::Value]] = Json::Value(true);
			divisionsJson.append(val);
		}
	}

	// save detections
	Json::Value& detectionsJson = root[JsonTypeNames[JsonTypes::DetectionResults]];
	for(auto iter : idToFlowGraphNodeMap_)
	{
		size_t value = graph.sumInFlow(iter.second.u);
		if(idToFlowGraphDivisionArcMap_.find(iter.first) != idToFlowGraphDivisionArcMap_.end())
			value -= flowMap[idToFlowGraphDivisionArcMap_[iter.first]];

		if(value > 0)
		{
			Json::Value val;
			val[JsonTypeNames[JsonTypes::Id]] = Json::Value((unsigned int)iter.first);
			val[JsonTypeNames[JsonTypes::Value]] = Json::Value((unsigned int)value);
			detectionsJson.append(val);
		}
	}

	output << root << std::endl;
}

JsonGraphReader::StateFeatureVector JsonGraphReader::extractFeatures(const Json::Value& entry, JsonTypes type)
{
	StateFeatureVector stateFeatVec;
	if(!entry.isMember(JsonTypeNames[type]))
		throw std::runtime_error("Could not find Json tags for " + JsonTypeNames[type]);

	const Json::Value featuresPerState = entry[JsonTypeNames[type]];

	if(!featuresPerState.isArray())
		throw std::runtime_error(JsonTypeNames[type] + " must be an array");

	if(!featuresPerState.size() > 0)
		throw std::runtime_error("Features may not be empty for " + JsonTypeNames[type]);

	// std::cout << "\tReading features for: " << JsonTypeNames[type] << std::endl;

	// get the features per state
	for(int i = 0; i < (int)featuresPerState.size(); i++)
	{
		// get features for the specific state
		FeatureVector featVec;
		const Json::Value& featuresForState = featuresPerState[i];

		if(!featuresForState.isArray())
			throw std::runtime_error("Expected to find a list of features for each state");

		if(!featuresForState.size() > 0)
		throw std::runtime_error("Features for state may not be empty for " + JsonTypeNames[type]);

		for(int j = 0; j < (int)featuresForState.size(); j++)
		{
			featVec.push_back(featuresForState[j].asDouble());
		}

		// std::cout << "\t\tfound " << featVec.size() << " features for state " << i << std::endl;

		stateFeatVec.push_back(featVec);
	}

	return stateFeatVec;
}

JsonGraphReader::FeatureVector JsonGraphReader::weightedSumOfFeatures(
	const StateFeatureVector& stateFeatures, 
	const FeatureVector& weights,
	size_t offset, 
	bool statesShareWeights)
{
	FeatureVector costPerState(stateFeatures.size(), 0.0);

	size_t weightIdx = offset;
	for(size_t state = 0; state < stateFeatures.size(); state++)
	{
		for(auto f : stateFeatures[state])
		{
			costPerState[state] = f * weights[(weightIdx++)];
		}

		if(statesShareWeights)
			weightIdx = offset;
	}

	return costPerState;
}

JsonGraphReader::FeatureVector JsonGraphReader::costsToScoreDeltas(const FeatureVector& costs)
{
	FeatureVector result;
	for(size_t i = 1; i < costs.size(); i++)
	{
		result.push_back(costs[i] - costs[i-1]);
		if(i > 1 && result[i-1] == result[i-2])
			std::cout << "Warning: found potentially problematic score setup: " << result[i-1] << " == " << result[i-2] << std::endl;
	}
	return result;
}

JsonGraphReader::ValueType JsonGraphReader::costsToScoreDelta(const FeatureVector& costs)
{
	assert(costs.size() == 2);
	return costs[0] - costs[1];
}


JsonGraphReader::FeatureVector JsonGraphReader::readWeightsFromJson(const std::string& filename)
{
	std::ifstream input(filename.c_str());
	if(!input.good())
		throw std::runtime_error("Could not open JSON weight file for reading: " + filename);

	Json::Value root;
	input >> root;

	if(!root.isMember(JsonTypeNames[JsonTypes::Weights]))
		throw std::runtime_error("Could not find 'Weights' group in JSON file");
	
	const Json::Value entry = root[JsonTypeNames[JsonTypes::Weights]];
	if(!entry.isArray())
		throw std::runtime_error("Cannot extract Weights from non-array JSON entry");

	FeatureVector weights;
	for(int i = 0; i < (int)entry.size(); i++)
	{
		weights.push_back(entry[i].asDouble());
	}
	return weights;
}
	
} // end namespace dpct
