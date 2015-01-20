#ifndef DPCT_NODE_H
#define DPCT_NODE_H

#include <vector>
#include <cstddef>

#include "userdata.h"

namespace dpct
{

class Arc;

class Node : public UserDataHolder
{
public:
	typedef std::vector<Arc*>::iterator ArcIt;

public:
	Node(const std::vector<double>& cellCountScoreDelta = {},
         UserDataPtr data = UserDataPtr());

	void increaseCellCount();

	void registerInArc(Arc* arc);
	void registerOutArc(Arc* arc);
    void registerObserverArc(Arc* arc);
    bool removeInArc(Arc* arc);
    bool removeOutArc(Arc* arc);

	void reset();
	void updateBestInArcAndScore();

	ArcIt getInArcsBegin()  { return inArcs_.begin(); }
	ArcIt getInArcsEnd()    { return inArcs_.end(); }
	ArcIt getOutArcsBegin() { return outArcs_.begin(); }
	ArcIt getOutArcsEnd()   { return outArcs_.end(); }
    Arc*  getBestInArc() const { return bestInArc_; }
    size_t getCellCount() const { return cellCount_; }
    double getCurrentScore() const { return currentScore_; }

protected:
    double getScoreDeltaForCurrentCellCount();
    void notifyObserverArcs();

protected:
	// things every node needs
	std::vector<Arc*> inArcs_;
	std::vector<Arc*> outArcs_;
    std::vector<Arc*> observerArcs_;
	size_t cellCount_; // do we also need to store how many cells are already dividing?
	Arc* bestInArc_;
	std::vector<double> cellCountScoreDelta_;
	double currentScore_;
};

} // namespace dpct


#endif // DPCT_NODE_H
