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
		 UserData* data):
	sourceNode_(source),
	targetNode_(target),
	type_(type),
	scoreDelta_(scoreDelta),
	currentScore_(scoreDelta),
	dependsOnCellInNode_(dependsOnCellInNode),
	data_(data)
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
}

void Arc::reset()
{
	currentScore_ = scoreDelta_;
	updateEnabledState();
}

void Arc::update()
{
	currentScore_ = scoreDelta_ + sourceNode_->getCurrentScore();
    std::cout << typeAsString() << "-Arc update: score is now " << currentScore_ << std::endl;
    updateEnabledState();
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
		enabled_ = dependsOnCellInNode_->getCellCount() > 0; // or == 1?
	}
	else
	{
		enabled_ = true;
	}
}

} // namespace dpct
