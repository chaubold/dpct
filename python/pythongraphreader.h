#ifndef PYHON_GRAPH_READER
#define PYHON_GRAPH_READER 

#include <boost/python.hpp>

#include "trackingalgorithm.h"
#include "log.h"
#include "graphbuilder.h"

#include "graphreader.h"

namespace dpct
{

// ----------------------------------------------------------------------------------------
/**
 * @brief A python graph reader provides functions to read a flow/magnusson graph from a python dict, 
 * and stores a mapping from ids to graph nodes, which allows to write back a result after tracking.
 * 
 */
class PythonGraphReader : public GraphReader
{
public:
	/**
	 * @brief Construct a python graph reader that reads from the specified files
	 * @param graphDict the python dictionary containing the graph description
	 * @param weightsDict the python dictionary containing the weights
	 * @param graphBuilder magnusson or flow graph builder
	 */
	PythonGraphReader(boost::python::dict& graphDict, boost::python::dict& weightsDict, GraphBuilder* graphBuilder);

	/**
	 * @brief Add nodes and arcs to the graph builder according to the model file. 
	 * Costs are computed from features times weights
	 */
	void createGraphFromPython();

	/**
	 * @brief Save a resulting flow map back to the python dict, the flow is extracted by the graph builder.
	 * 
	 * @returns Another python dictionary that looks exactly like the resulting JSON files.
	 */
	boost::python::object saveResult();

private:
	StateFeatureVector extractFeatures(boost::python::dict& entry, GraphReader::JsonTypes type);
	FeatureVector readWeightsFromPython(boost::python::dict& weightsDict);
	size_t getNumWeights(boost::python::dict& hypothesis, GraphReader::JsonTypes type, bool statesShareWeights);

private:
	/// python graph dictionary in the same style as if it was stored in a json file
	boost::python::dict& graphDict_;

	/// python dictionary containing the weights
	boost::python::dict& weightsDict_;
	
	/// the weight vector loaded from file
	FeatureVector weights_;
};

} // end namespace dpct

#endif // PYTHON_GRAPH_READER
