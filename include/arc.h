#ifndef DPCT_ARC_H
#define DPCT_ARC_H

#include <iostream>
#include "userdata.h"
#include "iarcnotifier.h"

namespace dpct
{

class Node;

// Arcs notify their observers when they are deleted!
class Arc : public IUserDataHolder, public IArcNotifier
{
public:
	enum Type{
		Appearance,
		Disappearance,
		Move,
		Division,
		Swap,
		Dummy
	};

    typedef std::function<Node*(Node*)> NodeMapFunc;

public:
	Arc() = delete;
    Arc(const Arc&) = delete;

	// creating an arc automatically registers it at source and target node!
	Arc(Node* source,
		Node* target,
		Type type,
		double scoreDelta,
		Node* dependsOnCellInNode = nullptr,
        UserDataPtr data = UserDataPtr()
		);

    Arc(const Arc& a,
        NodeMapFunc map_node,
        UserDataPtr data = UserDataPtr());

	double getCurrentScore() const { return currentScore_; }
	bool isEnabled() const { return enabled_; }

    // marking an arc as used has the consequence that each further appearance in a path
    // has a cost of zero, otherwise it will stay the same no matter how often it is used
    void markUsed(bool used = true);
    size_t getUseCount() const { return used_; }

	void reset();
	void update();
	void update(double additionalDelta);

    void changeTargetTo(Node* other);

	Node* getSourceNode() const { return sourceNode_; }
	Node* getTargetNode() const { return targetNode_; }
    Type getType() const { return type_; }
    double getScoreDelta() const { return ((used_>0)?0.0:scoreDelta_); }
    double getPlainScoreDelta() const { return scoreDelta_; }
    Node* getObservedNode() const { return dependsOnCellInNode_; }

    std::string typeAsString() const;

protected:
	void updateEnabledState();

protected:
	// things every arc needs
	Node* sourceNode_;
	Node* targetNode_;
	Type type_;
	double scoreDelta_;
	double currentScore_;
	bool enabled_;
	size_t used_;

	// dependencies for some arcs (e.g. divisions)
	Node* dependsOnCellInNode_;
};

} // namespace dpct


#endif // DPCT_ARC_H
