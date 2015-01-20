#include "arc.h"
#include "node.h"

#include <assert.h>

namespace dpct
{

Arc::Arc(Node* source,
         Node* target,
         Type type,
         double scoreDelta,
         Node* dependsOnCellInNode,
         UserDataPtr data):
    UserDataHolder(data),
	sourceNode_(source),
	targetNode_(target),
	type_(type),
	scoreDelta_(scoreDelta),
	currentScore_(scoreDelta),
    dependsOnCellInNode_(dependsOnCellInNode)
{
	assert(source != nullptr);
	assert(target != nullptr);

	if(type_ == Dummy)
	{
		assert(scoreDelta_ == 0.0);
	}

	updateEnabledState();

	sourceNode_->registerOutArc(this);
	targetNode_->registerInArc(this);

    if(dependsOnCellInNode_ != nullptr)
    {
        dependsOnCellInNode_->registerObserverArc(this);
    }
}

void Arc::reset()
{
	currentScore_ = scoreDelta_;
	updateEnabledState();
}

void Arc::update()
{
	currentScore_ = scoreDelta_ + sourceNode_->getCurrentScore();
    updateEnabledState();
    std::cout << typeAsString() << "-Arc update: score is now " << currentScore_ << " (enabled=" << (enabled_?"true":"false") << ")" << std::endl;
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
            std::cout << "Enabling formerly disabled " << typeAsString() << "-arc" << std::endl;
		enabled_ = dependsOnCellInNode_->getCellCount() > 0; // or == 1?
	}
	else
	{
        enabled_ = true;
	}
}

} // namespace dpct
