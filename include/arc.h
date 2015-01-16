#ifndef DPCT_ARC_H
#define DPCT_ARC_H

namespace dpct
{

class Node;
class UserData;

class Arc
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
		UserData* data = nullptr
		);

	double getCurrentScore() const { return currentScore_; }
	bool isEnabled() const { return enabled_; }
	const UserData* getUserData() const { return data_; }

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

	// how to store reference to hypotheses graph?
	UserData* data_;
};

} // namespace dpct


#endif // DPCT_ARC_H
