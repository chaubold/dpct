#include <limits>
#include <assert.h>

#include "node.h"
#include "arc.h"

namespace dpct
{

Node::Node(const std::vector<double>& cellCountScoreDelta,
           UserData* data):
    cellCountScoreDelta_(cellCountScoreDelta),
    data_(data),
    bestInArc_(nullptr),
    cellCount_(0),
    currentScore_(0.0)
{
    if(cellCountScoreDelta_.size() > 0)
    {
        currentScore_ = cellCountScoreDelta_[0];
    }
}

void Node::registerInArc(Arc* arc)
{
    inArcs_.push_back(arc);
    if(arc->isEnabled() &&
            (bestInArc_ == nullptr || arc->getCurrentScore() < bestInArc_->getCurrentScore()))
    {
        bestInArc_ = arc;
    }
}

void Node::registerOutArc(Arc* arc)
{
    outArcs_.push_back(arc);
}

void Node::reset()
{
    cellCount_ = 0;
    currentScore_ = getScoreDeltaForCurrentCellCount();
}

void Node::increaseCellCount()
{
    cellCount_++;
}

double Node::getScoreDeltaForCurrentCellCount()
{
    if(cellCountScoreDelta_.size() > cellCount_)
    {
        return cellCountScoreDelta_[cellCount_];
    }
    else
    {
        return 0.0;
    }
}

void Node::updateBestInArcAndScore()
{
    double bestScore = std::numeric_limits<double>::min();
    Arc* bestArc = nullptr;

    for(ArcIt it = inArcs_.begin(); it != inArcs_.end(); ++it)
    {
        if((*it)->getCurrentScore() > bestScore && (*it)->isEnabled())
        {
            bestScore = (*it)->getCurrentScore();
            bestArc = *it;
        }
    }

    if(bestArc != nullptr)
    {
        currentScore_ = bestScore + getScoreDeltaForCurrentCellCount();
        bestInArc_ = bestArc;
        std::cout << "Node update: score is now " << currentScore_ << std::endl;
    }
}


} // namespace dpct
