#include <deque>
#include <assert.h>
#include <functional>
#include <sstream>
#include <algorithm>

#include "trackingalgorithm.h"
#include "graph.h"
#include "node.h"
#include "arc.h"
#include "log.h"

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

    if(p.size() > 0)
        DEBUG_MSG("Path starts at node " << nodeName(p.front()->getSourceNode())
                  << " with cellCount " << p.front()->getSourceNode()->getCellCount()
                  << " and score " << p.front()->getSourceNode()->getCurrentScore());
    else
        return;

    for(Path::iterator it = p.begin(); it != p.end(); ++it)
    {
        DEBUG_MSG("\t follows " << (*it)->typeAsString() << " arc to node " <<  nodeName((*it)->getTargetNode())
                  << " with cellCount " <<  (*it)->getTargetNode()->getCellCount()
                  << " and score " <<  (*it)->getTargetNode()->getCurrentScore());
    }
}

void TrackingAlgorithm::tic()
{
    startTime_ = std::chrono::high_resolution_clock::now();
}

double TrackingAlgorithm::toc() // return time in seconds
{
    endTime_ = std::chrono::high_resolution_clock::now();
    return getElapsedSeconds();
}

double TrackingAlgorithm::getElapsedSeconds()
{
    std::chrono::duration<double> elapsed_seconds = endTime_ - startTime_;
    DEBUG_MSG("Elapsed time: " << elapsed_seconds.count() << "sec");
    return elapsed_seconds.count();
}

TrackingAlgorithm::Solution TrackingAlgorithm::translateToOriginGraph(TrackingAlgorithm::Solution &sol)
{
    Solution originSol;
    for(Path& p : sol)
    {
        Path originPath;
        originPath.reserve(p.size());

        for(Arc* a : p)
        {
            std::shared_ptr<ArcOriginData> arcOriginData = std::static_pointer_cast<ArcOriginData>(a->getUserData());
            std::shared_ptr<NodeOriginData> targetNodeOriginData = std::static_pointer_cast<NodeOriginData>(a->getTargetNode()->getUserData());

            assert(arcOriginData->getOriginsReverseOrder().size() == 1);
            originPath.push_back(arcOriginData->getOriginsReverseOrder().back());

            for(auto it = targetNodeOriginData->getConnectorsReverseOrder().rbegin();
                it != targetNodeOriginData->getConnectorsReverseOrder().rend();
                ++it)
            {
                originPath.push_back(*it);
            }
        }
        originSol.push_back(originPath);
    }
    return originSol;
}

} // namespace dpct
