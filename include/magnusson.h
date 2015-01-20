#ifndef DPCT_MAGNUSSON_H
#define DPCT_MAGNUSSON_H

#include "trackingalgorithm.h"
#include "userdata.h"

namespace dpct
{

// each swap arc keeps track of the arc it would cut if used
class MagnussonSwapArcUserData : public UserData
{
public:
    MagnussonSwapArcUserData(Arc* cutArc): arc_(cutArc) {}
    Arc* getCutArc() const { return arc_; }

    virtual std::string toString() const { return "Magnusson Swap Arc"; }
private:
    Arc* arc_;
};

// Klas Magnusson's cell tracking algorithm as in:
//  A batch algorithm using iterative application of the Viterbi algorithm to track cells and construct cell lineages @ ISBI 2012
//  Global linking of cell tracks using the Viterbi algorithm @ IEEE Transactions on Medical Imaging 2014
class Magnusson : public TrackingAlgorithm
{
public:
    Magnusson(Graph* graph, bool withSwap);

	virtual double track(std::vector<Path>& paths);
private:
	void updateNode(Node* n);
	void increaseCellCount(Node* n);
	void backtrack(Node* start, Path& p, TrackingAlgorithm::VisitorFunction nodeVisitor);

    //--------------------------------------
    // swap arc members
    bool withSwap_;
    Graph::ArcVector swapArcs_;
    // swap arc methods
    void insertSwapArcsForNewUsedPath(Path& p);
    void cleanUpUsedSwapArcs(Path& p);
    void removeSwapArcs();
};

} // namespace dpct

#endif // DPCT_MAGNUSSON_H
