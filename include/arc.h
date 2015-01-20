#ifndef DPCT_ARC_H
#define DPCT_ARC_H

#include <iostream>
#include "userdata.h"

namespace dpct
{

class Node;

class Arc : public UserDataHolder
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

public:
	Arc() = delete;

	// creating an arc automatically registers it at source and target node!
	Arc(Node* source,
		Node* target,
		Type type,
		double scoreDelta,
		Node* dependsOnCellInNode = nullptr,
        UserDataPtr data = UserDataPtr()
		);

	double getCurrentScore() const { return currentScore_; }
	bool isEnabled() const { return enabled_; }

	void reset();
	void update();

	Node* getSourceNode() const { return sourceNode_; }
	Node* getTargetNode() const { return targetNode_; }
    double getScoreDelta() const { return scoreDelta_; }

    std::string typeAsString();

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

	// dependencies for some arcs (e.g. divisions)
	Node* dependsOnCellInNode_;
};

} // namespace dpct


#endif // DPCT_ARC_H
