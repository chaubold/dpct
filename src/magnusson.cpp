#include <functional>
#include <algorithm>
#include <assert.h>

#include "magnusson.h"
#include "trackingalgorithm.h"
#include "graph.h"

using std::placeholders::_1;

namespace dpct
{

Magnusson::Magnusson(Graph* graph):
	TrackingAlgorithm(graph)
{}

double Magnusson::track(std::vector<TrackingAlgorithm::Path>& paths)
{
	paths.clear();
	double score = 0;

	// update scores from timestep 0 to the end
	for(size_t t = 0; t < graph_->getNumTimesteps(); ++t)
	{
		graph_->visitNodesInTimestep(t, std::bind(&Magnusson::updateNode, this, _1));
	}
    graph_->visitSpecialNodes(std::bind(&Magnusson::updateNode, this, _1));

    double scoreDelta = 0.0;

    do
    {
        // backtrack best path, increase cell counts -> invalidates scores!
        Path p;
        backtrack(&(graph_->getSinkNode()), p, std::bind(&Magnusson::increaseCellCount, this, _1));
        paths.push_back(p);
        std::cout << "Found path of length " << p.size() << " with score: " << p.back()->getCurrentScore() << std::endl;
        printPath(p);
        scoreDelta = p.back()->getCurrentScore();
        score += scoreDelta;

        // update scores along that path and all nodes that go away from there
        Node* firstPathNode = p.front()->getTargetNode();
        breadthFirstSearchVisitor(firstPathNode, std::bind(&Magnusson::updateNode, this, _1));
    } while(scoreDelta > 0 && paths.size() < 3);

    return score;
}

void Magnusson::updateNode(Node* n)
{
	n->updateBestInArcAndScore();
	double score = n->getCurrentScore();

	for(Node::ArcIt outArc = n->getOutArcsBegin(); outArc != n->getOutArcsEnd(); ++outArc)
	{
		(*outArc)->update();
	}
}

void Magnusson::increaseCellCount(Node* n)
{
    std::cout << "Increasing cell count of node " << n << std::endl;
	n->increaseCellCount();
}

void Magnusson::backtrack(Node* start, Path& p, TrackingAlgorithm::VisitorFunction nodeVisitor)
{
	p.clear();
	Node* current = start;

	while(current != &(graph_->getSourceNode()))
	{
        nodeVisitor(current);
		Arc* bestArc = current->getBestInArc();
		assert(bestArc != nullptr);

		p.push_back(bestArc);
		current = bestArc->getSourceNode();
	}

    nodeVisitor(current);
	std::reverse(p.begin(), p.end());
}

} // namespace dpct
