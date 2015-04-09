#ifndef FUSIONMOVE_H
#define FUSIONMOVE_H

#include "trackingalgorithm.h"

namespace dpct
{

class FusionMove
{
public:
    FusionMove(Graph* graph);

    std::shared_ptr<Graph> graphUnion(TrackingAlgorithm::Solution& sol_a,
                                      TrackingAlgorithm::Solution& sol_b);
protected:
    Graph* graph_;
};

} // namespace dpct

#endif // FUSIONMOVE_H
