#ifndef DPCT_MAGNUSSON_H
#define DPCT_MAGNUSSON_H

#include <functional>

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
// * A batch algorithm using iterative application of the
//     Viterbi algorithm to track cells and construct cell lineages
//     @ ISBI 2012
// * Global linking of cell tracks using the Viterbi algorithm
//     @ IEEE Transactions on Medical Imaging 2014
class Magnusson : public TrackingAlgorithm
{
public:
    typedef std::function<Arc*(Node*)> SelectorFunction;

public:
    Magnusson(Graph* graph, bool withSwap, bool usedArcsScoreZero = false);

    // specify a strategy to pick a path that starts from a node
    // through an arc.
    // defaults to using node->getBestInArc()
    void setPathStartSelectorFunction(SelectorFunction func);

    virtual double track(Solution& paths);
private:
	void updateNode(Node* n);
	void increaseCellCount(Node* n);
    void backtrack(Node* start,
                   Path& p,
                   TrackingAlgorithm::VisitorFunction nodeVisitor);

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

    SelectorFunction selectorFunction_;
};


// path selection strategies
Arc* selectBestInArc(Node* n);
Arc* selectSecondBestInArc(Node* n);
Arc* selectAtRandom(Node* n);

} // namespace dpct

#endif // DPCT_MAGNUSSON_H
