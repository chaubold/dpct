#include "arc.h"
#include "node.h"
#include "log.h"

#include <assert.h>

namespace dpct
{

Arc::Arc(Node* source,
         Node* target,
         Type type,
         double scoreDelta,
         Node* dependsOnCellInNode,
         UserDataPtr data):
    IUserDataHolder(data),
	sourceNode_(source),
	targetNode_(target),
	type_(type),
	scoreDelta_(scoreDelta),
	currentScore_(scoreDelta),
    dependsOnCellInNode_(dependsOnCellInNode),
    used_(0),
    enabled_(true)
{
	assert(source != nullptr);
	assert(target != nullptr);

	if(type_ == Dummy)
	{
		assert(scoreDelta_ == 0.0);
	}

	sourceNode_->registerOutArc(this);
	targetNode_->registerInArc(this);

    if(dependsOnCellInNode_ != nullptr)
    {
        dependsOnCellInNode_->registerObserverArc(this);
    }

    update();
}

Arc::Arc(const Arc& a,
        NodeMapFunc map_node,
        UserDataPtr data):
    Arc(map_node(a.sourceNode_),
        map_node(a.targetNode_),
        a.type_,
        a.scoreDelta_,
        map_node(a.dependsOnCellInNode_),
        data)
{}

void Arc::reset()
{
    used_ = 0;
	currentScore_ = scoreDelta_;
	updateEnabledState();
}

void Arc::update()
{
	currentScore_ = getScoreDelta() + sourceNode_->getCurrentScore();
    updateEnabledState();
    DEBUG_MSG(typeAsString() << "-Arc update: score is now " << currentScore_ << " (enabled=" << (enabled_?"true":"false") << ")");
}

void Arc::changeTargetTo(Node *other)
{
    targetNode_->removeInArc(this);
    targetNode_ = other;
    targetNode_->registerInArc(this);
}

std::string Arc::typeAsString()
{
    switch(type_)
    {
        case Appearance: return "Appearance";
        case Disappearance: return "Disappearance";
        case Move: return "Move";
        case Division: return "Division";
        case Swap: return "Swap";
        case Dummy: return "Dummy";
    }
}

void Arc::updateEnabledState()
{
	if(dependsOnCellInNode_ != nullptr)
	{
        if(enabled_ == false && dependsOnCellInNode_->getCellCount() > 0)
            DEBUG_MSG("Enabling formerly disabled " << typeAsString() << "-arc");
		enabled_ = dependsOnCellInNode_->getCellCount() > 0; // or == 1?
	}
	else
	{
        enabled_ = true;
	}
}

void Arc::markUsed(bool used)
{
    if(used)
        used_++;
    else
    {
        if(used_ == 0)
            throw std::runtime_error("Cannot reduce use count of arc that is not used!");
        used_--;
    }
}

} // namespace dpct
