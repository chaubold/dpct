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

	if(dependsOnCellInNode_ != nullptr)
	{
		enabled_ = dependsOnCellInNode_->getCellCount() > 0; // or == 1?
	}
	else
	{
		enabled_ = true;
	}

	sourceNode_->registerOutArc(this);
	targetNode_->registerInArc(this);
}

} // namespace dpct