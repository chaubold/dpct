#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python.hpp>

#include "pythongraphreader.h"
#include "flowgraph.h"
#include "flowgraphbuilder.h"
#include "graph.h"
#include "magnussongraphbuilder.h"
#include "magnusson.h"

using namespace dpct;
using namespace boost::python;

object flowBasedTracking(object& graphDict, object& weightsDict)
{
	dict graph = extract<dict>(graphDict);
	dict weights = extract<dict>(weightsDict);

	FlowGraph flowGraph;
    FlowGraphBuilder graphBuilder(&flowGraph);
	PythonGraphReader pyGraphReader(graph, weights, &graphBuilder);
	pyGraphReader.createGraphFromPython();
	flowGraph.maxFlowMinCostTracking();
	return pyGraphReader.saveResult();
}

object maxFlowTracking(object& graphDict, object& weightsDict)
{
	dict graph = extract<dict>(graphDict);
	dict weights = extract<dict>(weightsDict);

	FlowGraph flowGraph;
    FlowGraphBuilder graphBuilder(&flowGraph);
	PythonGraphReader pyGraphReader(graph, weights, &graphBuilder);
	pyGraphReader.createGraphFromPython();
	flowGraph.maxFlow();
	return pyGraphReader.saveResult();
}

object magnussonTracking(object& graphDict, object& weightsDict)
{
	dict graph = extract<dict>(graphDict);
	dict weights = extract<dict>(weightsDict);

	Graph::Configuration config(true, true, true);
	Graph magnussonGraph(config);
    MagnussonGraphBuilder graphBuilder(&magnussonGraph);
    PythonGraphReader pyGraphReader(graph, weights, &graphBuilder);
	pyGraphReader.createGraphFromPython();

    Magnusson tracker(&magnussonGraph, true, true, false);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);
    std::cout << "\nTracking finished in " << tracker.getElapsedSeconds() 
    		  << " secs with energy " << -score << std::endl;
    graphBuilder.getSolutionFromPaths(paths);
	return pyGraphReader.saveResult();
}

/**
 * @brief Python interface of 'dpct' module
 */
BOOST_PYTHON_MODULE( dpct )
{
	def("trackFlowBased", flowBasedTracking, args("graph", "weights"),
		"Use the flow-based tracker on a graph specified as a dictionary,"
		"in the same structure as the supported JSON format. Similarly, the weights are also given as dict.\n\n"
		"Returns a python dictionary similar to the result.json file, but also stores 'value' or 'divisionValue'"
		"for each detection and link.");
	def("trackMaxFlow", maxFlowTracking, args("graph", "weights"),
		"Run min-cost max-flow tracking on a graph specified as a dictionary,"
		"in the same structure as the supported JSON format. Similarly, the weights are also given as dict.\n\n"
		"The max-flow disregards division constraints and simply pushes as much flow through the net as possible.\n\n"
		"Returns a python dictionary similar to the result.json file, but also stores 'value' or 'divisionValue'"
		"for each detection and link.");
	def("trackMagnusson", magnussonTracking, args("graph", "weights"),
		"Use Magnusson's tracker on a graph specified as a dictionary,"
		"in the same structure as the supported JSON format. Similarly, the weights are also given as dict.\n\n"
		"Magnusson only approximates the residual graph and is thus much faster but not as close to the optimum, "
		"but still always feasible.\n\n"
		"Returns a python dictionary similar to the result.json file, but also stores 'value' or 'divisionValue'"
		"for each detection and link.");
}
