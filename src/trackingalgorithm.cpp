#include <deque>
#include <assert.h>
#include <functional>
#include <sstream>
#include <algorithm>

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
            Node* next = (*outArc)->getTargetNode();
            assert(next != nullptr);

            // only add if not yet present in queue (or should we remove it and only update at the end?)
            if(std::find(queue.begin(), queue.end(), next) == queue.end())
            {
                queue.push_back(next);
            }
		}
	}
}

void TrackingAlgorithm::printPath(TrackingAlgorithm::Path& p)
{
    std::function<std::string(Node*)> nodeName = [](Node* n)
    {
        std::stringstream s;
        if(n->getUserData() != nullptr)
        {
            s << n->getUserData() << " = " << n;
        }
        else
        {
            s << n;
        }
        return s.str();
    };

    if(p.size() > 0);
        std::cout << "Path starts at node " << nodeName(p.front()->getSourceNode())
                  << " with cellCount " << p.front()->getSourceNode()->getCellCount()
                  << " and score " << p.front()->getSourceNode()->getCurrentScore() << std::endl;

    for(Path::iterator it = p.begin(); it != p.end(); ++it)
    {
        std::cout << "\t follows " << (*it)->typeAsString() << " arc to node " <<  nodeName((*it)->getTargetNode())
                  << " with cellCount " <<  (*it)->getTargetNode()->getCellCount()
                  << " and score " <<  (*it)->getTargetNode()->getCurrentScore() << std::endl;
    }
}

} // namespace dpct
