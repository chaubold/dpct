#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python.hpp>

#include "pythongraphreader.h"
#include "flowgraph.h"
#include "flowgraphbuilder.h"

using namespace dpct;
using namespace boost::python;

object flowTracking(object& graphDict, object& weightsDict)
{
	dict graph = extract<dict>(graphDict);
	dict weights = extract<dict>(weightsDict);

	FlowGraph flowgraph;
    FlowGraphBuilder graphBuilder(&flowgraph);
	PythonGraphReader pyGraphReader(graph, weights, &graphBuilder);
	pyGraphReader.createGraphFromPython();
	flowgraph.maxFlowMinCostTracking();
	return pyGraphReader.saveResult();
}

/**
 * @brief Python interface of 'dpct' module
 */
BOOST_PYTHON_MODULE( dpct )
{
	def("track", flowTracking, args("graph", "weights"),
		"Use the flow-based tracker on a graph specified as a dictionary,"
		"in the same structure as the supported JSON format. Similarly, the weights are also given as dict."
		"Returns a python dictionary similar to the result.json file, but also stores 'value' or 'divisionValue'"
		"for each detection and link.");
}
