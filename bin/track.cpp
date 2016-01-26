#include <iostream>

#include <boost/program_options.hpp>

#include "flowgraph.h"
#include "jsongraphreader.h"

using namespace dpct;

int main(int argc, char** argv) {
	namespace po = boost::program_options;

	std::string modelFilename;
	std::string weightsFilename;
	std::string outputFilename;

	// Declare the supported options.
	po::options_description description("Allowed options");
	description.add_options()
	    ("help", "produce help message")
	    ("model,m", po::value<std::string>(&modelFilename), "filename of model stored as Json file")
	    ("weights,w", po::value<std::string>(&weightsFilename), "filename of the weights stored as Json file")
	    ("output,o", po::value<std::string>(&outputFilename), "filename where the resulting tracking (as links) will be stored as Json file")
	;

	po::variables_map variableMap;
	po::store(po::parse_command_line(argc, argv, description), variableMap);
	po::notify(variableMap);

	if (variableMap.count("help")) 
	{
	    std::cout << description << std::endl;
	    return 1;
	}

	if (!variableMap.count("model") || !variableMap.count("output") || !variableMap.count("weights")) 
	{
	    std::cout << "Model, Weights and Output filenames have to be specified!" << std::endl;
	    std::cout << description << std::endl;
	} 
	else 
	{
	    FlowGraph graph;
	    FlowGraphBuilder graphBuilder(&graph);
	    JsonGraphReader jsonReader(modelFilename, weightsFilename, &graphBuilder);
	    jsonReader.createGraphFromJson();
	    std::cout << "Model has state zero energy: " << jsonReader.getInitialStateEnergy() << std::endl;
	    graph.maxFlowMinCostTracking(jsonReader.getInitialStateEnergy());
	    jsonReader.saveResultJson(outputFilename);
	}
}
