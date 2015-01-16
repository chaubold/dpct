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


} // namespace dpct