#ifndef DPCT_NODE_H
#define DPCT_NODE_H

#include "config.hpp"

#include <vector>
#include <cstddef>
#include <limits>
#include <memory>

#include "userdata.h"
#include "iarcnotifier.h"

namespace dpct
{

class Arc;

// Nodes notify observers when the cellcount increases
class Node : public IUserDataHolder, public IArcNotifier, public std::enable_shared_from_this<Node>
{
public:
	typedef std::vector<Arc*>::iterator ArcIt;
    typedef std::vector<Arc*>::const_iterator ConstArcIt;
    typedef std::function<void(Arc*)> VisitorFunction;
    typedef std::function<void(const Arc*)> ConstVisitorFunction;

public:
    Node() = delete;
    Node(const Node&) = delete;

	explicit Node(const std::vector<double>& cellCountScore = {},
         UserDataPtr data = UserDataPtr());
    // this dedicated copy constructor does NOT copy connected arcs, but only node internals
	Node(const Node& n,
         UserDataPtr data = UserDataPtr());

	DPCT_API void increaseCellCount();
	DPCT_API  void addToCellCountScore(size_t state, double score);

	DPCT_API void registerInArc(Arc* arc);
	DPCT_API void registerOutArc(Arc* arc);
	DPCT_API bool removeInArc(Arc* arc);
	DPCT_API bool removeOutArc(Arc* arc);

	DPCT_API void reset();
	DPCT_API void updateBestInArcAndScore();

	DPCT_API ArcIt getInArcsBegin()  { return inArcs_.begin(); }
	DPCT_API ArcIt getInArcsEnd()    { return inArcs_.end(); }
	DPCT_API ArcIt getOutArcsBegin() { return outArcs_.begin(); }
	DPCT_API ArcIt getOutArcsEnd()   { return outArcs_.end(); }
	DPCT_API ConstArcIt getInArcsBegin() const { return inArcs_.begin(); }
	DPCT_API ConstArcIt getInArcsEnd()   const { return inArcs_.end(); }
	DPCT_API ConstArcIt getOutArcsBegin()const { return outArcs_.begin(); }
	DPCT_API ConstArcIt getOutArcsEnd()  const { return outArcs_.end(); }
	DPCT_API void visitInArcs(const VisitorFunction& func);
	DPCT_API void visitInArcs (const ConstVisitorFunction& func) const;
	DPCT_API void visitOutArcs(const VisitorFunction& func);
	DPCT_API void visitOutArcs (const ConstVisitorFunction& func) const;

	DPCT_API Arc* getAppearanceArc() const    { return appearanceArc_; }
	DPCT_API Arc* getDisappearanceArc() const { return disappearanceArc_; }

	DPCT_API Arc*  getBestInArc() const     { return bestInArc_; }
	DPCT_API size_t getCellCount() const    { return cellCount_; }
	DPCT_API double getCurrentScore() const { return currentScore_; }
	DPCT_API size_t getNumStates() const {return cellCountScore_.size(); }

	DPCT_API size_t getNumInArcs() const  { return inArcs_.size(); }
	DPCT_API size_t getNumOutArcs() const { return outArcs_.size(); }

	DPCT_API size_t getNumActiveDivisions() const { return numActiveDivisions_; }
	DPCT_API void increaseNumActiveDivisions(int count = 1) { numActiveDivisions_ += count; }

	DPCT_API size_t getMoveInArcUsedSum() const  { return numUsedMoveInArcs_; }
	DPCT_API size_t getMoveOutArcUsedSum() const { return numUsedMoveOutArcs_; }
	DPCT_API void increaseNumUsedOutArcs(int count = 1) { numUsedMoveOutArcs_ += count; }
	DPCT_API void increaseNumUsedInArcs (int count = 1) { numUsedMoveInArcs_ += count; }

	DPCT_API friend std::ostream& operator<<(std::ostream& lhs, const Node& rhs);

	DPCT_API double getScoreDeltaForCurrentCellCount();
	DPCT_API std::shared_ptr<Node> getSharedPtr(){ return shared_from_this(); }

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

    // cache states
    size_t numActiveDivisions_;
    size_t numUsedMoveInArcs_;
    size_t numUsedMoveOutArcs_;
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
