#include <limits>
#include <assert.h>
#include <stdexcept>

#include "node.h"
#include "arc.h"
#include "userdata.h"

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
    if(cellCountScoreDelta_.size() > 1)
    {
        currentScore_ = cellCountScoreDelta_[1] - cellCountScoreDelta_[0];
    }
    else if(cellCountScoreDelta_.size() == 1)
    {
        throw std::runtime_error("Node - Constructor: Cannot use a cellCountScoreDelta table with only one state, need at least two!");
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

void Node::registerObserverArc(Arc *arc)
{
    observerArcs_.push_back(arc);
}

void Node::reset()
{
    cellCount_ = 0;
    currentScore_ = getScoreDeltaForCurrentCellCount();
}

void Node::increaseCellCount()
{
    cellCount_++;
    notifyObserverArcs();
}

double Node::getScoreDeltaForCurrentCellCount()
{
    if(cellCountScoreDelta_.size() > cellCount_ + 1)
    {
        return cellCountScoreDelta_[cellCount_ + 1] - cellCountScoreDelta_[cellCount_];
    }
    else if(cellCountScoreDelta_.size() > 0)
    {
        return std::numeric_limits<double>::lowest();
    }
    else
    {
        return 0.0;
    }
}

void Node::notifyObserverArcs()
{
    for(ArcIt it = observerArcs_.begin(); it != observerArcs_.end(); ++it)
    {
        (*it)->update();
    }
}

void Node::updateBestInArcAndScore()
{
    double bestScore = std::numeric_limits<double>::lowest();
    Arc* bestArc = nullptr;

    std::cout << "Updating node with " << inArcs_.size() << " inArcs and " << outArcs_.size() << " out arcs" << std::endl;

    for(ArcIt it = inArcs_.begin(); it != inArcs_.end(); ++it)
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
        if(data_ != nullptr)
        {
            std::cout << "Node (" << *((NameData*)data_) << ") update: score is now " << currentScore_ << std::endl;
        }
        else
        {
            std::cout << "Node update: score is now " << currentScore_ << std::endl;
        }
    }
}


} // namespace dpct
