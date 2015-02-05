#ifndef DPCT_MAGNUSSON_H
#define DPCT_MAGNUSSON_H

#include "trackingalgorithm.h"
#include "userdata.h"

namespace dpct
{

// each swap arc keeps track of the arc it would cut if used,
// the arc that the path A that chose the swap arc would take,
// and the arc that the previous path B will be redirected along
class MagnussonSwapArcUserData : public UserData
{
public:
    MagnussonSwapArcUserData(Arc* cutArc,
                             Arc* replacementA,
                             Arc* replacementB):
        arc_(cutArc),
        replacementA_(replacementA),
        replacementB_(replacementB)
    {}
    Arc* getCutArc() const { return arc_; }
    Arc* getReplacementAArc() const { return replacementA_; }
    Arc* getReplacementBArc() const { return replacementB_; }

    virtual std::string toString() const { return "Magnusson Swap Arc"; }
private:
    Arc* arc_;
    Arc* replacementA_;
    Arc* replacementB_;
};

// Klas Magnusson's cell tracking algorithm as in:
//  A batch algorithm using iterative application of the Viterbi algorithm to track cells and construct cell lineages @ ISBI 2012
//  Global linking of cell tracks using the Viterbi algorithm @ IEEE Transactions on Medical Imaging 2014
class Magnusson : public TrackingAlgorithm
{
public:
    Magnusson(Graph* graph, bool withSwap, bool usedArcsScoreZero = false);

	virtual double track(std::vector<Path>& paths);
private:
	void updateNode(Node* n);
	void increaseCellCount(Node* n);
	void backtrack(Node* start, Path& p, TrackingAlgorithm::VisitorFunction nodeVisitor);

    //--------------------------------------
    // swap arc members
    bool withSwap_;
    bool usedArcsScoreZero_;
    std::vector<Arc*> swapArcs_;
    // swap arc methods
    void insertSwapArcsForNewUsedPath(Path& p);
    void cleanUpUsedSwapArcs(Path& p, std::vector<Path> &paths);
    void removeSwapArcs();
    void removeSwapArc(Arc* a);
    void removeArc(Arc *a);
};

} // namespace dpct

#endif // DPCT_MAGNUSSON_H
