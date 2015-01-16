#ifndef DPCT_NODE_H
#define DPCT_NODE_H

#include <vector>
#include <cstddef>

namespace dpct
{

class Arc;
class UserData;

class Node
{
public:
	Node(const std::vector<double>& cellCountScoreDelta = {},
		 UserData* data = nullptr);

	size_t getCellCount() const { return cellCount_; }
	void registerInArc(Arc* arc);
	void registerOutArc(Arc* arc);
	const UserData* getUserData() const { return data_; }

protected:
	// things every node needs
	std::vector<Arc*> inArcs_;
	std::vector<Arc*> outArcs_;
	size_t cellCount_; // do we also need to store how many cells are already dividing?
	Arc* bestInArc_;
	std::vector<double> cellCountScoreDelta_;
	double currentScore_;

	// how to store reference to hypotheses graph?
	UserData* data_;
};

} // namespace dpct


#endif // DPCT_NODE_H
