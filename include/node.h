#ifndef DPCT_NODE_H
#define DPCT_NODE_H

#include <vector>
#include <cstddef>
#include <limits>

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
    typedef std::function<void(Arc*)> VisitorFunction;
    typedef std::function<void(const Arc*)> ConstVisitorFunction;

public:
    Node() = delete;
    Node(const Node&) = delete;

    Node(const std::vector<double>& cellCountScore = {},
         UserDataPtr data = UserDataPtr());
    // this dedicated copy constructor does NOT copy connected arcs, but only node internals
    Node(const Node& n,
         UserDataPtr data = UserDataPtr());

	void increaseCellCount();
    void addToCellCountScore(size_t state, double score);

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
    void visitInArcs(const VisitorFunction& func);
    void visitInArcs (const ConstVisitorFunction& func) const;
    void visitOutArcs(const VisitorFunction& func);
    void visitOutArcs (const ConstVisitorFunction& func) const;

    size_t getMoveInArcUsedSum() const;
    size_t getMoveOutArcUsedSum() const;
    Arc* getAppearanceArc() const    { return appearanceArc_; }
    Arc* getDisappearanceArc() const { return disappearanceArc_; }

    Arc*  getBestInArc() const     { return bestInArc_; }
    size_t getCellCount() const    { return cellCount_; }
    double getCurrentScore() const { return currentScore_; }
    size_t getNumStates() const {return cellCountScore_.size(); }

    size_t getNumInArcs() const  { return inArcs_.size(); }
    size_t getNumOutArcs() const { return outArcs_.size(); }

    void accumulateScoreDelta(Node* other);
    void addArcCost(Arc *other, bool usedArcsScoreZero);

    friend std::ostream& operator<<(std::ostream& lhs, const Node& rhs);

protected:
    double getScoreDeltaForCurrentCellCount();

protected:
	// things every node needs
	std::vector<Arc*> inArcs_;
	std::vector<Arc*> outArcs_;
    Arc* appearanceArc_;
    Arc* disappearanceArc_;
	size_t cellCount_; // do we also need to store how many cells are already dividing?
	Arc* bestInArc_;
    std::vector<double> cellCountScore_;
	double currentScore_;
};

std::ostream& operator<<(std::ostream& lhs, const Node& rhs);

inline double Node::getScoreDeltaForCurrentCellCount()
{
    if(cellCountScore_.size() > cellCount_ + 1)
    {
        return cellCountScore_[cellCount_ + 1] - cellCountScore_[cellCount_];
    }
    else if(cellCountScore_.size() > 0)
    {
        return std::numeric_limits<double>::lowest();
    }
    else
    {
        return 0.0;
    }
}

} // namespace dpct


#endif // DPCT_NODE_H
