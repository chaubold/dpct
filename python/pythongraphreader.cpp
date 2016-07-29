#include "pythongraphreader.h"
#include <assert.h>
#include <fstream>

using namespace boost::python;

namespace dpct
{

PythonGraphReader::PythonGraphReader(boost::python::dict& graphDict, boost::python::dict& weightsDict, GraphBuilder* graphBuilder):
	GraphReader(graphBuilder),
	graphDict_(graphDict),
	weightsDict_(weightsDict)
{
}

size_t PythonGraphReader::getNumWeights(boost::python::dict& hypothesis, GraphReader::JsonTypes type, bool statesShareWeights)
{
	size_t numWeights;
	StateFeatureVector stateFeatVec = extractFeatures(hypothesis, type);
	
	if(statesShareWeights)
		numWeights = stateFeatVec[0].size();
	else
	{
		numWeights = 0;
		for(auto stateFeats : stateFeatVec)
			numWeights += stateFeats.size();
	}
	return numWeights;
}

void PythonGraphReader::createGraphFromPython()
{
	
	// get flag whether states should share weights or not
	bool statesShareWeights = false;
	if(graphDict_.has_key(GraphReader::JsonTypeNames[GraphReader::JsonTypes::Settings]))
	{
		dict settings = extract<dict>(graphDict_[GraphReader::JsonTypeNames[GraphReader::JsonTypes::Settings]]);
		if(settings.has_key(GraphReader::JsonTypeNames[GraphReader::JsonTypes::StatesShareWeights]))
			statesShareWeights = extract<bool>(settings[GraphReader::JsonTypeNames[GraphReader::JsonTypes::StatesShareWeights]]);
	}

	// ------------------------------------------------------------------------------
	// get weight vector and number of weights needed for each different variable type
	FeatureVector weights = readWeightsFromPython(weightsDict_);
	size_t numDetWeights = 0;
	size_t numDivWeights = 0;
	size_t numAppWeights = 0;
	size_t numDisWeights = 0;
	size_t numLinkWeights = 0;

	list segmentationHypotheses = extract<list>(graphDict_[GraphReader::JsonTypeNames[GraphReader::JsonTypes::Segmentations]]);
	for(size_t i = 0; (int)i < len(segmentationHypotheses); i++)
	{
		dict jsonHyp = extract<dict>(segmentationHypotheses[i]);
		numDetWeights = getNumWeights(jsonHyp, GraphReader::JsonTypes::Features, statesShareWeights);

		if(jsonHyp.has_key(GraphReader::JsonTypeNames[GraphReader::JsonTypes::DivisionFeatures]))
			numDivWeights = getNumWeights(jsonHyp, GraphReader::JsonTypes::DivisionFeatures, statesShareWeights);

		if(jsonHyp.has_key(GraphReader::JsonTypeNames[GraphReader::JsonTypes::AppearanceFeatures]))
			numAppWeights = getNumWeights(jsonHyp, GraphReader::JsonTypes::AppearanceFeatures, statesShareWeights);

		if(jsonHyp.has_key(GraphReader::JsonTypeNames[GraphReader::JsonTypes::DisappearanceFeatures]))
			numDisWeights = getNumWeights(jsonHyp, GraphReader::JsonTypes::DisappearanceFeatures, statesShareWeights);
	}

	list linkingHypotheses = extract<list>(graphDict_[GraphReader::JsonTypeNames[GraphReader::JsonTypes::Links]]);
	for(size_t i = 0; (int)i < len(linkingHypotheses); i++)
	{
		dict jsonHyp = extract<dict>(linkingHypotheses[i]);
		if(jsonHyp.has_key(GraphReader::JsonTypeNames[GraphReader::JsonTypes::Features]))
				numLinkWeights = getNumWeights(jsonHyp, GraphReader::JsonTypes::Features, statesShareWeights);
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
	std::cout << "\tcontains " << len(segmentationHypotheses) << " segmentation hypotheses" << std::endl;
	
	for(size_t i = 0; (int)i < len(segmentationHypotheses); i++)
	{
		dict jsonHyp = extract<dict>(segmentationHypotheses[i]);
		
		if(!jsonHyp.has_key(GraphReader::JsonTypeNames[GraphReader::JsonTypes::Id]))
			throw std::runtime_error("Cannot read detection hypothesis without Id!");
		size_t id = extract<size_t>(jsonHyp[GraphReader::JsonTypeNames[GraphReader::JsonTypes::Id]]);

		if(!jsonHyp.has_key(GraphReader::JsonTypeNames[GraphReader::JsonTypes::Features]))
			throw std::runtime_error("Cannot read detection hypothesis without features!");

		if(jsonHyp.has_key(GraphReader::JsonTypeNames[GraphReader::JsonTypes::Timestep]))
		{
			list timeJson = extract<list>(jsonHyp[GraphReader::JsonTypeNames[GraphReader::JsonTypes::Timestep]]);
			if(len(timeJson) != 2)
				throw std::runtime_error("Node's Timestep is supposed to be a 2-element array");
			auto timeRange = std::make_pair(extract<int>(timeJson[0])(), extract<int>(timeJson[1])());
			graphBuilder_->setNodeTimesteps(id, timeRange);
		}

		FeatureVector detCosts = weightedSumOfFeatures(extractFeatures(jsonHyp, GraphReader::JsonTypes::Features), weights, detWeightOffset, statesShareWeights);
		FeatureVector detCostDeltas = costsToScoreDeltas(detCosts);
		FeatureVector appearanceCostDeltas;
		FeatureVector disappearanceCostDeltas;
		if(jsonHyp.has_key(GraphReader::JsonTypeNames[GraphReader::JsonTypes::AppearanceFeatures]))
			appearanceCostDeltas = costsToScoreDeltas(weightedSumOfFeatures(extractFeatures(jsonHyp, GraphReader::JsonTypes::AppearanceFeatures), weights, appWeightOffset, statesShareWeights));

		if(jsonHyp.has_key(GraphReader::JsonTypeNames[GraphReader::JsonTypes::DisappearanceFeatures]))
			disappearanceCostDeltas = costsToScoreDeltas(weightedSumOfFeatures(extractFeatures(jsonHyp, GraphReader::JsonTypes::DisappearanceFeatures), weights, disWeightOffset, statesShareWeights));

		size_t targetIdx = 0;
		if(jsonHyp.has_key(GraphReader::JsonTypeNames[GraphReader::JsonTypes::DisappearanceTarget]))
		{
			targetIdx = extract<size_t>(jsonHyp[GraphReader::JsonTypeNames[GraphReader::JsonTypes::DisappearanceTarget]]);
		}

		graphBuilder_->addNode(id, detCosts, detCostDeltas, appearanceCostDeltas, disappearanceCostDeltas, targetIdx);
	}

	// read linking hypotheses
	std::cout << "\tcontains " << len(linkingHypotheses) << " linking hypotheses" << std::endl;
	for(size_t i = 0; (int)i < len(linkingHypotheses); i++)
	{
		dict jsonHyp = extract<dict>(linkingHypotheses[i]);

		size_t srcId = extract<size_t>(jsonHyp[GraphReader::JsonTypeNames[GraphReader::JsonTypes::SrcId]]);
		size_t destId = extract<size_t>(jsonHyp[GraphReader::JsonTypeNames[GraphReader::JsonTypes::DestId]]);
		graphBuilder_->addArc(srcId, destId, costsToScoreDeltas(weightedSumOfFeatures(extractFeatures(jsonHyp, GraphReader::JsonTypes::Features), weights, linkWeightOffset, statesShareWeights)));
	}

	// read divisions
	for(size_t i = 0; (int)i < len(segmentationHypotheses); i++)
	{
		dict jsonHyp = extract<dict>(segmentationHypotheses[i]);
		size_t id = extract<size_t>(jsonHyp[GraphReader::JsonTypeNames[GraphReader::JsonTypes::Id]]);

		if(jsonHyp.has_key(GraphReader::JsonTypeNames[GraphReader::JsonTypes::DivisionFeatures]))
		{
			graphBuilder_->allowMitosis(id, costsToScoreDelta(weightedSumOfFeatures(extractFeatures(jsonHyp, GraphReader::JsonTypes::DivisionFeatures), weights, divWeightOffset, statesShareWeights)));
		}
	}

	// read exclusion constraints between detections
	// const Json::Value exclusions = graphDict_[GraphReader::JsonTypeNames[GraphReader::JsonTypes::Exclusions]];
	// std::cout << "\tcontains " << exclusions.size() << " exclusions" << std::endl;
	// for(size_t i = 0; i < (int)exclusions.size(); i++)
	// {
	// 	const Json::Value jsonExc = exclusions[i];
	// 	// TODO: implement some API for those?!
	// }
	if(graphDict_.has_key(GraphReader::JsonTypeNames[GraphReader::JsonTypes::Exclusions]) && len(graphDict_[GraphReader::JsonTypeNames[GraphReader::JsonTypes::Exclusions]]) > 0)
		throw std::runtime_error("FlowSolver cannot deal with exclusion constraints yet!");
}

object PythonGraphReader::saveResult()
{
	list detectionResults;
	list linkResults;
	list divisionResults;

	// save links
	GraphBuilder::ArcValueMap arcValues = graphBuilder_->getArcValues();
	list linkingHypotheses = extract<list>(graphDict_[GraphReader::JsonTypeNames[GraphReader::JsonTypes::Links]]);
	for(size_t i = 0; (int)i < len(linkingHypotheses); i++)
	{
		// store in graph
		dict jsonHyp = extract<dict>(linkingHypotheses[i]);
		size_t srcId = extract<size_t>(jsonHyp[GraphReader::JsonTypeNames[GraphReader::JsonTypes::SrcId]]);
		size_t destId = extract<size_t>(jsonHyp[GraphReader::JsonTypeNames[GraphReader::JsonTypes::DestId]]);
		size_t value = arcValues[std::make_pair(srcId, destId)];
		jsonHyp[GraphReader::JsonTypeNames[GraphReader::JsonTypes::Value]] = value;

		// store in extra list
		dict linkRes;
		linkRes[JsonTypeNames[JsonTypes::SrcId]] = srcId;
		linkRes[JsonTypeNames[JsonTypes::DestId]] = destId;
		linkRes[JsonTypeNames[JsonTypes::Value]] = value;
		linkResults.append(linkRes);
	}

	// save divisions and detections
	GraphBuilder::DivisionValueMap divisionValues = graphBuilder_->getDivisionValues();
	GraphBuilder::NodeValueMap nodeValues = graphBuilder_->getNodeValues();
	
	list segmentationHypotheses = extract<list>(graphDict_[GraphReader::JsonTypeNames[GraphReader::JsonTypes::Segmentations]]);
	for(size_t i = 0; (int)i < len(segmentationHypotheses); i++)
	{
		// store in graph
		dict jsonHyp = extract<dict>(segmentationHypotheses[i]);
		size_t id = extract<size_t>(jsonHyp[GraphReader::JsonTypeNames[GraphReader::JsonTypes::Id]]);
		bool division = divisionValues[id];
		size_t value = nodeValues[id];
		jsonHyp[GraphReader::JsonTypeNames[GraphReader::JsonTypes::Value]] = value;
		jsonHyp[GraphReader::JsonTypeNames[GraphReader::JsonTypes::DivisionValue]] = division;

		// store in extra list
		dict detRes;
		detRes[JsonTypeNames[JsonTypes::Id]] = id;
		detRes[JsonTypeNames[JsonTypes::Value]] = value;
		detectionResults.append(detRes);

		dict divRes;
		divRes[JsonTypeNames[JsonTypes::Id]] = id;
		divRes[JsonTypeNames[JsonTypes::Value]] = division;
		divisionResults.append(divRes);
	}

	dict result;
	result[GraphReader::JsonTypeNames[GraphReader::JsonTypes::DetectionResults]] = detectionResults;
	result[GraphReader::JsonTypeNames[GraphReader::JsonTypes::LinkResults]] = linkResults;
	result[GraphReader::JsonTypeNames[GraphReader::JsonTypes::DivisionResults]] = divisionResults;
	return result;
}

PythonGraphReader::StateFeatureVector PythonGraphReader::extractFeatures(boost::python::dict& entry, GraphReader::JsonTypes type)
{
	StateFeatureVector stateFeatVec;
	if(!entry.has_key(GraphReader::JsonTypeNames[type]))
		throw std::runtime_error("Could not find dict entry for " + GraphReader::JsonTypeNames[type]);

	list featuresPerState = extract<list>(entry[GraphReader::JsonTypeNames[type]]);

	if(len(featuresPerState) == 0)
		throw std::runtime_error("Features may not be empty for " + GraphReader::JsonTypeNames[type]);

	// std::cout << "\tReading features for: " << GraphReader::JsonTypeNames[type] << std::endl;

	// get the features per state
	for(size_t i = 0; (int)i < len(featuresPerState); i++)
	{
		// get features for the specific state
		FeatureVector featVec;
		list featuresForState = extract<list>(featuresPerState[i]);

		
		if(len(featuresForState) ==  0)
			throw std::runtime_error("Features for state may not be empty for " + GraphReader::JsonTypeNames[type]);

		for(size_t j = 0; (int)j < len(featuresForState); j++)
		{
			featVec.push_back(extract<double>(featuresForState[j]));
		}

		// std::cout << "\t\tfound " << featVec.size() << " features for state " << i << std::endl;

		stateFeatVec.push_back(featVec);
	}

	return stateFeatVec;
}

PythonGraphReader::FeatureVector PythonGraphReader::readWeightsFromPython(boost::python::dict& weightsDict)
{
	list weightsList = extract<list>(weightsDict[GraphReader::GraphReader::JsonTypeNames[GraphReader::GraphReader::JsonTypes::Weights]]);
	FeatureVector weights;
	for(size_t i = 0; (int)i < len(weightsList); i++)
	{
		weights.push_back(extract<GraphBuilder::ValueType>(weightsList[i]));
	}
	return weights;
}
	
} // end namespace dpct
