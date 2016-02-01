#include "arc.h"
#include "node.h"
#include "log.h"

#include "flowgraph.h"
#include "magnusson.h"
#include <assert.h>

namespace dpct
{

Arc::Arc(Node* source,
         Node* target,
         Type type,
         const std::vector<double>& scoreDeltas,
         Node* dependsOnCellInNode,
         UserDataPtr data):
    IUserDataHolder(data),
	sourceNode_(source),
	targetNode_(target),
	type_(type),
	scoreDeltas_(scoreDeltas),
	currentScore_(scoreDeltas[0]),
    dependsOnCellInNode_(dependsOnCellInNode),
    used_(0),
    enabled_(true)
{
	assert(source != nullptr);
	assert(target != nullptr);

	if(type_ == Dummy)
	{
        for(size_t d : scoreDeltas_)
		  assert(d == 0.0);
	}

	sourceNode_->registerOutArc(this);
	targetNode_->registerInArc(this);

    if(dependsOnCellInNode_ != nullptr)
    {
        dependsOnCellInNode_->registerObserverArc(this);
    }

    updateEnabledState();
    update();
}

Arc::Arc(const Arc& a,
        NodeMapFunc map_node,
        UserDataPtr data):
    Arc(map_node(a.sourceNode_),
        map_node(a.targetNode_),
        a.type_,
        a.scoreDeltas_,
        map_node(a.dependsOnCellInNode_),
        data)
{}

void Arc::reset()
{
    used_ = 0;
	currentScore_ = scoreDeltas_[0];
	updateEnabledState();
}

void Arc::update()
{
    currentScore_ = getScoreDelta() + sourceNode_->getCurrentScore();
    // updateEnabledState();
    DEBUG_MSG(typeAsString() << "-Arc update: score is now " << currentScore_ << " (enabled="
               << (enabled_?"true":"false") << ")" << " scoreDelta=" << scoreDeltas_);
}

void Arc::update(double additionalDelta)
{
	currentScore_ = getScoreDelta() + sourceNode_->getCurrentScore() + additionalDelta;
    // updateEnabledState();
    DEBUG_MSG(typeAsString() << "-Arc update: score is now " << currentScore_ << " (enabled="
               << (enabled_?"true":"false") << ")" << " scoreDelta=" << scoreDeltas_);
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
	switch(type_)
    {
        case Division:
        {
            assert(dependsOnCellInNode_ != nullptr);
            if(used_ > 0)
            {
                // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!! disabling division because it was already used!" << std::endl;
                enabled_ = false;
            }
            else
            {
                enabled_ = dependsOnCellInNode_->getCellCount() == 1
                			&& (dependsOnCellInNode_->getDisappearanceArc() == nullptr || dependsOnCellInNode_->getDisappearanceArc()->getUseCount() == 0)
                			&& (targetNode_->getAppearanceArc() == nullptr || targetNode_->getAppearanceArc()->getUseCount() == 0)
                			&& dependsOnCellInNode_->getNumActiveDivisions() == 0
                			&& dependsOnCellInNode_->getMoveOutArcUsedSum() == 1;
                if(enabled_)
                {
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
                            }
                            return;
                        }
                    }
                }
            }
        } break;
        case Swap:
        {
            // check that the link that this swap should exchange with is still used:
            Arc* arcToCut = std::static_pointer_cast<MagnussonSwapArcUserData>(this->getUserData())->getCutArc();
            enabled_ = arcToCut->used_ > 0 && arcToCut->getSourceNode()->getNumActiveDivisions() == 0;
    	    // check that replacement arcs are enabled
    	    enabled_ &= std::static_pointer_cast<MagnussonSwapArcUserData>(this->getUserData())->getReplacementAArc()->isEnabled()
		                  && std::static_pointer_cast<MagnussonSwapArcUserData>(this->getUserData())->getReplacementBArc()->isEnabled();
        } break;
        case Appearance:
        {
            enabled_ = targetNode_->getMoveInArcUsedSum() == 0;
        } break;
        case Disappearance:
        {
            enabled_ = sourceNode_->getMoveOutArcUsedSum() == 0 && sourceNode_->getNumActiveDivisions() == 0;
        } break;
        case Move:
        {
            enabled_ = (targetNode_->getAppearanceArc() == nullptr || targetNode_->getAppearanceArc()->getUseCount() == 0)
                        && (sourceNode_->getDisappearanceArc() == nullptr || sourceNode_->getDisappearanceArc()->getUseCount() == 0)
                        && sourceNode_->getNumActiveDivisions() == 0;
        } break;
    	default:
    	{
            enabled_ = true;
        }
    }
}

void Arc::markUsed(bool used)
{
    static auto arcEnabler = [](Arc* a){
        a->updateEnabledState();
    };

    if(used)
    {
        used_++;
    }
    else
    {
        if(used_ == 0)
            throw std::runtime_error("Cannot reduce use count of arc that is not used!");
        used_--;
    }

    int count = used ? 1 : -1;

    if(type_ == Division)
    {
        if(used_)
        {
        	if(dependsOnCellInNode_->getCellCount() != 1)
        		throw std::runtime_error("Using division arc where mother cell does not contain exactly one cell");
        	if(dependsOnCellInNode_->getNumActiveDivisions() != 0)
        		throw std::runtime_error("Using division arc where mother cell already has active divisions");
        }
        dependsOnCellInNode_->increaseNumActiveDivisions(count);
        targetNode_->increaseNumUsedInArcs(count);
        dependsOnCellInNode_->visitObserverArcs(arcEnabler);
        dependsOnCellInNode_->visitOutArcs(arcEnabler);
        targetNode_->visitInArcs(arcEnabler);
    }
    else if(type_ == Move)
    {
        sourceNode_->increaseNumUsedOutArcs(count);
        targetNode_->increaseNumUsedInArcs(count);
        sourceNode_->visitOutArcs(arcEnabler);
        sourceNode_->visitObserverArcs(arcEnabler);
        targetNode_->visitInArcs(arcEnabler);
    }
    else if(type_ == Appearance)
    {
        targetNode_->visitInArcs(arcEnabler);
    }
    else if(type_ == Disappearance)
    {
        sourceNode_->visitOutArcs(arcEnabler);
        sourceNode_->visitObserverArcs(arcEnabler);
    }
    updateEnabledState();

    // update swap arcs if any!
    visitObserverArcs(arcEnabler);
}

} // namespace dpct
