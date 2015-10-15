#include <limits>
#include <assert.h>
#include <stdexcept>
#include <algorithm>

#include "node.h"
#include "arc.h"
#include "userdata.h"
#include "log.h"

namespace dpct
{

Node::Node(const std::vector<double>& cellCountScore,
           UserDataPtr data):
    IUserDataHolder(data),
    cellCountScore_(cellCountScore),
    bestInArc_(nullptr),
    appearanceArc_(nullptr),
    disappearanceArc_(nullptr),
    cellCount_(0),
    currentScore_(0.0)
{
    if(cellCountScore_.size() > 1)
    {
        currentScore_ = cellCountScore_[1] - cellCountScore_[0];
    }
    else if(cellCountScore_.size() == 1)
    {
        throw std::runtime_error("Node - Constructor: Cannot use a cellCountScoreDelta table with only one state, need at least two!");
    }
}

Node::Node(const Node& n, UserDataPtr data):
    Node(n.cellCountScore_, data)
{}

void Node::registerInArc(Arc* arc)
{
    if(arc->getType() == Arc::Appearance)
    {
        if(appearanceArc_ != nullptr)
            throw std::runtime_error("Node can not have two appearance arcs!");
        appearanceArc_ = arc;
    }
    inArcs_.push_back(arc);
}

void Node::registerOutArc(Arc* arc)
{
    if(arc->getType() == Arc::Disappearance)
    {
        if(disappearanceArc_ != nullptr)
            throw std::runtime_error("Node can not have two disappearance arcs!");
        disappearanceArc_ = arc;
    }
    outArcs_.push_back(arc);
}

bool Node::removeInArc(Arc *arc)
{
    if(arc == appearanceArc_)
        appearanceArc_ = nullptr;

    ArcIt it = std::find(inArcs_.begin(), inArcs_.end(), arc);
    if(it != inArcs_.end())
    {
        inArcs_.erase(it);
        return true;
    }
    return false;
}

bool Node::removeOutArc(Arc *arc)
{
    if(arc == disappearanceArc_)
        disappearanceArc_ = nullptr;

    DEBUG_MSG("Trying to remove out arc from a vector of size: " << outArcs_.size());
    ArcIt it = std::find(outArcs_.begin(), outArcs_.end(), arc);
    if(it != outArcs_.end())
    {
        outArcs_.erase(it);
        return true;
    }
    return false;
}
void Node::reset()
{
    cellCount_ = 0;
    currentScore_ = getScoreDeltaForCurrentCellCount();
}

void Node::increaseCellCount()
{
    cellCount_++;
    // TODO: remove the following as Graph::visitNodesPerTimestep performs the update anyway?
    visitObserverArcs([](Arc* a){ a->update(); });
}

void Node::addToCellCountScore(size_t state, double score)
{
    assert(cellCountScore_.size() > state);
    cellCountScore_[state] += score;
    updateBestInArcAndScore();
}

void Node::updateBestInArcAndScore()
{
    double bestScore = std::numeric_limits<double>::lowest();
    Arc* bestArc = nullptr;

    DEBUG_MSG("Updating node with " << inArcs_.size() << " inArcs and " << outArcs_.size() << " out arcs");

    for(ConstArcIt it = inArcs_.begin(); it != inArcs_.end(); ++it)
    {
        double cs = (*it)->getCurrentScore();
        if((cs > bestScore || bestArc == nullptr) && (*it)->isEnabled())
        {
            bestScore = cs;
            bestArc = *it;
        }
    }

    if(bestArc != nullptr)
    {
        currentScore_ = bestScore + getScoreDeltaForCurrentCellCount();
        bestInArc_ = bestArc;
#ifdef DEBUG_LOG
        if(getUserData())
        {
            DEBUG_MSG("Node (" << *(std::static_pointer_cast<NameData>(getUserData())) << ") update: score is now " << currentScore_);
        }
        else
        {
            DEBUG_MSG("Node update: score is now " << currentScore_);
        }
#endif
    }
}

void Node::accumulateScoreDelta(Node *other)
{
    assert(other->cellCountScore_.size() == cellCountScore_.size());
    for(size_t i = 0; i < cellCountScore_.size(); i++)
    {
        cellCountScore_[i] += other->cellCountScore_[i];
    }
}

void Node::addArcCost(Arc* other, bool usedArcsScoreZero)
{
    size_t numStatesToChange = std::min<size_t>(1, cellCountScore_.size());
    if(!usedArcsScoreZero)
        numStatesToChange = cellCountScore_.size();

    for(size_t i = 1; i < numStatesToChange; i++)
    {
        cellCountScore_[i] += other->getPlainScoreDelta();
    }
}

std::ostream& operator<<(std::ostream& lhs, const Node& rhs)
{
    lhs << "Node with scores(";
    for(double score: rhs.cellCountScore_)
    {
        lhs << score << ", ";
    }
    lhs << "), " << rhs.inArcs_.size() << " in-arcs and "
        << rhs.outArcs_.size() << " out arcs";
    return lhs;
}

} // namespace dpct
