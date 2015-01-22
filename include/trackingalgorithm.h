#ifndef DPCT_TRACKING_ALGORITHM_H
#define DPCT_TRACKING_ALGORITHM_H

#include <vector>
#include <functional>
#include <memory>
#include <chrono>

#include "node.h"
#include "arc.h"
#include "graph.h"

namespace dpct
{

class TrackingAlgorithm
{
public:
    typedef std::vector<Arc*> Path;
	typedef std::function<void(Node*)> VisitorFunction;

public:
	TrackingAlgorithm(Graph* graph);

	// track cells, return overall score, and fill 'paths' vector with all chosen paths
	virtual double track(std::vector<Path>& paths) = 0;

    void printPath(TrackingAlgorithm::Path &p);
protected:
	Graph* graph_;
	std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;

	void breadthFirstSearchVisitor(Node* begin, VisitorFunction func);
	void tic();
	double toc(); // return time in seconds
};

} // namespace dpct

#endif // DPCT_TRACKING_ALGORITHM_H
