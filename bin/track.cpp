#include <iostream>

#include <boost/program_options.hpp>

#include "graph.h"
#include "magnusson.h"
#include "flowgraph.h"
#include "jsongraphreader.h"
#include "flowgraphbuilder.h"
#include "magnussongraphbuilder.h"

using namespace dpct;

int main(int argc, char** argv) {
	namespace po = boost::program_options;

	std::string modelFilename;
	std::string weightsFilename;
	std::string outputFilename;
	std::string method("flow");
	bool swap = true;
	bool useOrderedNodeListInBF = true;
	bool partialBFUpdates = true;
	size_t maxNumPaths = 0;

	// Declare the supported options.
	po::options_description description("Allowed options");
	description.add_options()
	    ("help", "produce help message")
	    ("model,m", po::value<std::string>(&modelFilename), "filename of model stored as Json file")
	    ("weights,w", po::value<std::string>(&weightsFilename), "filename of the weights stored as Json file")
	    ("output,o", po::value<std::string>(&outputFilename), "filename where the resulting tracking (as links) will be stored as Json file")
	    ("method,e", po::value<std::string>(&method), "method to use for tracking: 'flow' (default), 'flow-flow', 'magnusson-flow' or 'magnusson'")
	    ("swap,s", po::value<bool>(&swap), "whether swap arcs are enabled (default=true)")
	    ("maxNumPaths,n", po::value<size_t>(&maxNumPaths), "maximum number of paths to find, default=0=no limit")
	    ("orderNodes", po::value<bool>(&useOrderedNodeListInBF), "use ordered node list in BF? flow only. (default=true)")
	    ("partialBF", po::value<bool>(&partialBFUpdates), "check which parts of the graph were influenced by last path and only update there? flow only. (default=true)")
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
		if(method == "flow")
		{
		    FlowGraph graph;
		    FlowGraphBuilder graphBuilder(&graph);
		    JsonGraphReader jsonReader(modelFilename, weightsFilename, &graphBuilder);
		    jsonReader.createGraphFromJson();
		    std::cout << "Model has state zero energy: " << jsonReader.getInitialStateEnergy() << std::endl;
		    graph.maxFlowMinCostTracking(jsonReader.getInitialStateEnergy(), swap, maxNumPaths, useOrderedNodeListInBF, partialBFUpdates);
		    jsonReader.saveResultJson(outputFilename);
		}
		else if(method == "flow-flow")
		{
		    FlowGraph graph;
		    FlowGraphBuilder graphBuilder(&graph);
		    JsonGraphReader jsonReader(modelFilename, weightsFilename, &graphBuilder);
		    jsonReader.createGraphFromJson();
		    std::cout << "Model has state zero energy: " << jsonReader.getInitialStateEnergy() << std::endl;
		    double energy = graph.maxFlowMinCostTracking(jsonReader.getInitialStateEnergy(), false, maxNumPaths, useOrderedNodeListInBF, partialBFUpdates);
		    graph.maxFlowMinCostTracking(energy, true, maxNumPaths, useOrderedNodeListInBF, partialBFUpdates);
		    jsonReader.saveResultJson(outputFilename);
		}
		else if(method == "magnusson")
		{
			Graph::Configuration config(true, true, true);
    		Graph graph(config);
		    MagnussonGraphBuilder graphBuilder(&graph);
		    JsonGraphReader jsonReader(modelFilename, weightsFilename, &graphBuilder);
		    jsonReader.createGraphFromJson();
		    std::cout << "Model has state zero energy: " << jsonReader.getInitialStateEnergy() << std::endl;

		    Magnusson tracker(&graph, swap, true, false);
		    if(maxNumPaths > 0)
		    	tracker.setMaxNumberOfPaths(maxNumPaths);
		    
		    std::vector<TrackingAlgorithm::Path> paths;
		    double score = tracker.track(paths);
		    std::cout << "\nTracking finished in " << tracker.getElapsedSeconds() << " secs with score " 
		    		  << jsonReader.getInitialStateEnergy() - score << std::endl;
		    graphBuilder.getSolutionFromPaths(paths);
		    jsonReader.saveResultJson(outputFilename);
		}
		else if(method == "magnusson-flow")
		{
			double zeroEnergy, score;

			// set up magnusson
			Graph::Configuration config(true, true, true);
    		Graph graph(config);
		    MagnussonGraphBuilder graphBuilder(&graph);
		    std::vector<TrackingAlgorithm::Path> paths;

		    { // scope needed due to weird model scores that show up otherwise
			    JsonGraphReader jsonReader(modelFilename, weightsFilename, &graphBuilder);
			    jsonReader.createGraphFromJson();
			    zeroEnergy = jsonReader.getInitialStateEnergy();
			    std::cout << "Model has state zero energy: " << zeroEnergy << std::endl;

			    // track magnusson
			    Magnusson tracker(&graph, swap, true, false);
			    if(maxNumPaths > 0)
			    	tracker.setMaxNumberOfPaths(maxNumPaths);
			    
			    score = tracker.track(paths);
			    std::cout << "\nTracking finished in " << tracker.getElapsedSeconds() << " secs with score " 
			    		  << zeroEnergy - score << std::endl;

			    std::cout << "Extracting solution" << std::endl;
			}

		    // set up flow
		    FlowGraph flowGraph;
		    FlowGraphBuilder flowGraphBuilder(&flowGraph);
		    JsonGraphReader flowJsonReader(modelFilename, weightsFilename, &flowGraphBuilder);
		    flowJsonReader.createGraphFromJson();
		    std::cout << "Model has state zero energy: " << flowJsonReader.getInitialStateEnergy() << std::endl;

		    // initialize flow with magnusson's result
		    std::cout << "initializing flow solver" << std::endl;
		    flowGraph.initializeResidualGraph(true, useOrderedNodeListInBF);

		    auto flowPaths = graphBuilder.translateSolution(paths, flowGraphBuilder);
		    for(auto p : flowPaths)
		    {
		    	flowGraph.augmentUnitFlow(p);
		    	flowGraph.updateEnabledArcs(p);
		    }
		    flowGraph.synchronizeDivisionDuplicateArcFlows();

		    // track flow
		    std::cout << "beginning tracking" << std::endl;
			flowGraph.maxFlowMinCostTracking(zeroEnergy - score, true, maxNumPaths, useOrderedNodeListInBF, partialBFUpdates);
		    flowJsonReader.saveResultJson(outputFilename);
		}
		else
			throw std::runtime_error("Unknown tracking method selected");
	}
}
