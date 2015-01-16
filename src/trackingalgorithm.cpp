#include <deque>
#include <assert.h>

#include "trackingalgorithm.h"
#include "graph.h"
#include "node.h"
#include "arc.h"

namespace dpct
{

TrackingAlgorithm::TrackingAlgorithm(Graph* graph):
	graph_(graph)
{

}

void TrackingAlgorithm::breadthFirstSearchVisitor(Node* begin, VisitorFunction func)
{
    assert(begin != nullptr);
	std::deque<Node*> queue;
	queue.push_back(begin);

	while(queue.size() > 0)
	{
        Node* node = queue.front();
		queue.pop_front();
        assert(node != nullptr);
        func(node);

		// add all nodes following outgoing arcs to queue
		for(Node::ArcIt outArc = node->getOutArcsBegin(); outArc != node->getOutArcsEnd(); ++outArc)
		{
			queue.push_back((*outArc)->getTargetNode());
            assert(queue.back() != nullptr);
		}
	}
}

void TrackingAlgorithm::printPath(TrackingAlgorithm::Path& p)
{
    if(p.size() > 0);
        std::cout << "Path starts at node " << p.front()->getSourceNode()
                  << " with cellCount " << p.front()->getSourceNode()->getCellCount()
                  << " and score " << p.front()->getSourceNode()->getCurrentScore() << std::endl;

    for(Path::iterator it = p.begin(); it != p.end(); ++it)
    {
        std::cout << "\t follows " << (*it)->typeAsString() << " arc to node " <<  (*it)->getTargetNode()
                  << " with cellCount " <<  (*it)->getTargetNode()->getCellCount()
                  << " and score " <<  (*it)->getTargetNode()->getCurrentScore() << std::endl;
    }
}

} // namespace dpct
