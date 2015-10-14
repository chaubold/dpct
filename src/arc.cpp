#include "arc.h"
#include "node.h"
#include "log.h"

#include "magnusson.h"
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
    DEBUG_MSG(typeAsString() << "-Arc update: score is now " << currentScore_ << " (enabled=" 
               << (enabled_?"true":"false") << ")" << " scoreDelta=" << scoreDelta_);
}

void Arc::update(double additionalDelta)
{
	currentScore_ = getScoreDelta() + sourceNode_->getCurrentScore() + additionalDelta;
    updateEnabledState();
    DEBUG_MSG(typeAsString() << "-Arc update: score is now " << currentScore_ << " (enabled=" 
               << (enabled_?"true":"false") << ")" << " scoreDelta=" << scoreDelta_);
}

void Arc::changeTargetTo(Node *other)
{
    targetNode_->removeInArc(this);
    targetNode_ = other;
    targetNode_->registerInArc(this);
}

std::string Arc::typeAsString() const
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
#ifdef DEBUG_LOG
        if(enabled_ == false && dependsOnCellInNode_->getCellCount() > 0)
            DEBUG_MSG("Enabling formerly disabled " << typeAsString() << "-arc");
#endif
		if(type_ == Division)
        {
            if(used_ > 0)
            {
                // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!! disabling division because it was already used!" << std::endl;
                enabled_ = false;
            }
            else
            {
                enabled_ = dependsOnCellInNode_->getCellCount() == 1;
                if(enabled_)
                {
                    size_t activeOutArcs = 0;
                    
                    // make sure that there is no active arc between the mother and daughter candidates yet
                    for(Node::ConstArcIt outArcIt = dependsOnCellInNode_->getOutArcsBegin(); 
                        outArcIt != dependsOnCellInNode_->getOutArcsEnd(); 
                        ++outArcIt)
                    {
                        if((*outArcIt)->getTargetNode() == targetNode_)
                        {
                            if((*outArcIt)->getUseCount() > 0)
                            {
                                // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!! disabling division because the very same link has been used as transition!" << std::endl;
                                enabled_ = false;
                                return;
                            }
                        }

                        activeOutArcs += (*outArcIt)->getUseCount();
                    }

                    // make sure that we have only one active out arc
                    if(activeOutArcs != 1)
                    {
                        enabled_ = false;
                        // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!! disabling division because mother node has " << activeOutArcs << " active outgoing arcs" << std::endl;
                    }
                }
            }
        }
        else
        {
            enabled_ = dependsOnCellInNode_->getCellCount() > 0; // or == 1?
        }
	}
    else if(type_ == Swap)
    {
        // check that the link that this swap should exchange with is still used:
        Arc* arcToCut = std::static_pointer_cast<MagnussonSwapArcUserData>(this->getUserData())->getCutArc();
        enabled_ = arcToCut->used_ > 0;
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
    updateEnabledState();
}

} // namespace dpct
