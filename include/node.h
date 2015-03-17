#ifndef DPCT_NODE_H
#define DPCT_NODE_H

#include <vector>
#include <cstddef>

#include "userdata.h"
#include "iarcnotifier.h"

namespace dpct
{

class Arc;

// Nodes notify observers when the cellcount increases
class Node : public IUserDataHolder, public IArcNotifier
{
public:
	typedef std::vector<Arc*>::iterator ArcIt;
    typedef std::vector<Arc*>::const_iterator ConstArcIt;

public:
    Node() = delete;
    Node(const Node&) = delete;

    Node(const std::vector<double>& cellCountScoreDelta = {},
         UserDataPtr data = UserDataPtr());
    // this dedicated copy constructor does NOT copy connected arcs, but only node internals
    Node(const Node& n,
         UserDataPtr data = UserDataPtr());

	void increaseCellCount();

	void registerInArc(Arc* arc);
	void registerOutArc(Arc* arc);
    bool removeInArc(Arc* arc);
    bool removeOutArc(Arc* arc);

	void reset();
	void updateBestInArcAndScore();

	ArcIt getInArcsBegin()  { return inArcs_.begin(); }
	ArcIt getInArcsEnd()    { return inArcs_.end(); }
	ArcIt getOutArcsBegin() { return outArcs_.begin(); }
	ArcIt getOutArcsEnd()   { return outArcs_.end(); }
    ConstArcIt getInArcsBegin() const { return inArcs_.begin(); }
    ConstArcIt getInArcsEnd()   const { return inArcs_.end(); }
    ConstArcIt getOutArcsBegin()const { return outArcs_.begin(); }
    ConstArcIt getOutArcsEnd()  const { return outArcs_.end(); }

    Arc*  getBestInArc() const     { return bestInArc_; }
    size_t getCellCount() const    { return cellCount_; }
    double getCurrentScore() const { return currentScore_; }

    size_t getNumInArcs() const  { return inArcs_.size(); }
    size_t getNumOutArcs() const { return outArcs_.size(); }

    void accumulateScoreDelta(Node* other);
    void addArcCost(Arc *other, bool usedArcsScoreZero);

protected:
    double getScoreDeltaForCurrentCellCount();

protected:
	// things every node needs
	std::vector<Arc*> inArcs_;
	std::vector<Arc*> outArcs_;
	size_t cellCount_; // do we also need to store how many cells are already dividing?
	Arc* bestInArc_;
	std::vector<double> cellCountScoreDelta_;
	double currentScore_;
};

} // namespace dpct


#endif // DPCT_NODE_H
