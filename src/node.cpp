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
    appearanceArc_(nullptr),
    disappearanceArc_(nullptr),
    cellCount_(0),
    bestInArc_(nullptr),
    cellCountScore_(cellCountScore),
    currentScore_(0.0),
    numActiveDivisions_(0),
    numUsedMoveInArcs_(0),
    numUsedMoveOutArcs_(0)
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
    visitObserverArcs([](Arc* a){ 
        a->updateEnabledState();
        a->update(); 
    });
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
    else
    {
        // bestInArc_ = nullptr;
        currentScore_ = std::numeric_limits<double>::lowest();
        if(getUserData())
        {
            DEBUG_MSG("Node (" << *(std::static_pointer_cast<NameData>(getUserData())) << ") update: no good in-arc!");
        }
        else
        {
            DEBUG_MSG("Node update: no good in-arc!");
        }
    }
}

// size_t Node::getMoveInArcUsedSum() const
// {
//     size_t sum = 0;
    
//     for(const auto& arc : inArcs_)
//     {
//         if(arc->getType() == Arc::Move || arc->getType() == Arc::Division)
//             sum += arc->getUseCount();
//     }

//     return sum;
// }

// size_t Node::getMoveOutArcUsedSum() const
// {
//     size_t sum = 0;
    
//     for(const auto& arc : outArcs_)
//     {
//         if(arc->getType() == Arc::Move)
//             sum += arc->getUseCount();
//     }

//     return sum;
// }

void Node::visitInArcs(const VisitorFunction& func)
{
    for(auto a : inArcs_)
        func(a);
}

void Node::visitInArcs (const ConstVisitorFunction& func) const
{
    for(auto a : inArcs_)
        func(a);
}

void Node::visitOutArcs(const VisitorFunction& func)
{
    for(auto a : outArcs_)
        func(a);
}

void Node::visitOutArcs (const ConstVisitorFunction& func) const
{
    for(auto a : outArcs_)
        func(a);
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
