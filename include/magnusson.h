#ifndef DPCT_MAGNUSSON_H
#define DPCT_MAGNUSSON_H

#include "trackingalgorithm.h"

namespace dpct
{

class Magnusson : public TrackingAlgorithm
{
public:
	Magnusson(Graph* graph);

	virtual double track(std::vector<Path>& paths);

private:
	void updateNode(Node* n);
	void increaseCellCount(Node* n);
	void backtrack(Node* start, Path& p, TrackingAlgorithm::VisitorFunction nodeVisitor);
};

} // namespace dpct

#endif // DPCT_MAGNUSSON_H