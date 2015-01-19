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

    while(true)
    {
        // backtrack best path, increase cell counts -> invalidates scores!
        Path p;
        backtrack(&(graph_->getSinkNode()), p, std::bind(&Magnusson::increaseCellCount, this, _1));
        std::cout << "Current best path of length " << p.size() << " has score: " << p.back()->getCurrentScore() << std::endl;
        printPath(p);
        scoreDelta = p.back()->getCurrentScore();

        // only continue if this path adds to the overall score
        if(scoreDelta < 0)
        {
            std::cout << "Path has negative reward, stopping here with a total number of " << paths.size() << " cells added" << std::endl;
            break;
        }

        score += scoreDelta;
        paths.push_back(p);

        // update scores along that path and all nodes that go away from there
        Node* firstPathNode = p.front()->getTargetNode();
        if(firstPathNode->getUserData() != nullptr)
        {
            std::cout << "Beginning update at node: " << *((NameData*)firstPathNode->getUserData()) << std::endl;
        }
        else
        {
            std::cout << "Beginning update at node " << firstPathNode << std::endl;
        }
        breadthFirstSearchVisitor(firstPathNode, std::bind(&Magnusson::updateNode, this, _1));
    };

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
    if(n->getUserData() != nullptr)
    {
        std::cout << "Increasing cell count of node " << *((NameData*)n->getUserData()) << " = " << n << std::endl;
    }
    else
    {
        std::cout << "Increasing cell count of node " << n << std::endl;
    }
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
