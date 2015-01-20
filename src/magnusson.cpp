#include <functional>
#include <algorithm>
#include <assert.h>

#include "magnusson.h"
#include "trackingalgorithm.h"
#include "graph.h"

using std::placeholders::_1;

namespace dpct
{

Magnusson::Magnusson(Graph* graph, bool withSwap):
    TrackingAlgorithm(graph),
    withSwap_(withSwap)
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

        // insert swap arcs
        if(withSwap_)
        {
            insertSwapArcsForNewUsedPath(p);
        }

        // update scores along that path and all nodes that go away from there
        Node* firstPathNode = p.front()->getTargetNode();
        if(firstPathNode->getUserData())
        {
            std::cout << "Beginning update at node: " << *(std::static_pointer_cast<NameData>(firstPathNode->getUserData())) << std::endl;
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
    if(n->getUserData())
    {
        std::cout << "Increasing cell count of node " << *(std::static_pointer_cast<NameData>(n->getUserData())) << " = " << n << std::endl;
    }
    else
    {
        std::cout << "Increasing cell count of node " << n << std::endl;
    }
	n->increaseCellCount();
}

void Magnusson::backtrack(Node* start, TrackingAlgorithm::Path& p, TrackingAlgorithm::VisitorFunction nodeVisitor)
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

void Magnusson::insertSwapArcsForNewUsedPath(TrackingAlgorithm::Path &p)
{
    /* Swap Arc Insertion: walk along path
     * - find alternative incoming arcs of the traversed node, where there are also multiple outgoing arcs of the previous node
     * - insert swap arc between any alternative pair of nodes connected to the path by in / out arcs.
    */
    for(Path::iterator it = p.begin(); it != p.end(); ++it)
    {
        Node *source = (*it)->getSourceNode();
        Node *target = (*it)->getTargetNode();

        for(Node::ArcIt outIt = source->getOutArcsBegin(); outIt != source->getOutArcsEnd(); ++outIt)
        {
            Node *alternativeTarget = (*outIt)->getTargetNode();
            if(alternativeTarget == target)
                continue;

            for(Node::ArcIt inIt = target->getInArcsBegin(); inIt != target->getInArcsEnd(); ++inIt)
            {
                Node *alternativeSource = (*inIt)->getSourceNode();
                if(alternativeSource == source)
                    continue;

                // found a candidate
                double score = (*outIt)->getScoreDelta() + (*inIt)->getScoreDelta() - (*it)->getScoreDelta();
                // if there is an arc connecting the two alternative nodes, subtract its score delta as well
                for(Node::ArcIt alternativeOutIt = alternativeSource->getOutArcsBegin(); alternativeOutIt != alternativeSource->getOutArcsEnd(); ++alternativeOutIt)
                {
                    if( (*alternativeOutIt)->getTargetNode() == alternativeTarget
                            && (*alternativeOutIt)->getType() != Arc::Appearance // only swap Moves (or swapped moves) for now
                            && (*alternativeOutIt)->getType() != Arc::Disappearance
                            && (*alternativeOutIt)->getType() != Arc::Division) // && (*alternativeOutIt)->getType() != Arc::Swap) // is that needed? -> shouldn't be, could be "swapped" twice
                        score -= (*alternativeOutIt)->getScoreDelta();
                }

                // the swap arc does not depend on other nodes being part of a path,
                // as this algorithm never removes cells and thus the previously populated nodes can be used in swaps.
                // BUT: it needs to store a reference to the arc that it would cut, and this action is performed in cleanUpUsedSwapArcs()
                Graph::ArcPtr arc(new Arc(alternativeSource,
                                          alternativeTarget,
                                          Arc::Swap,
                                          score,
                                          nullptr,
                                          std::make_shared<MagnussonSwapArcUserData>(*it)
                                          ));

                std::cout << "!!! Adding SWAP ARC between " << alternativeSource << " and " << alternativeTarget << " with score " << score << std::endl;

                swapArcs_.push_back(arc);
            }
        }
    }
}

void Magnusson::cleanUpUsedSwapArcs(TrackingAlgorithm::Path &p)
{
    // if a swap arc was used, we can find the path that was affected by this and create the two paths after swapping
    for(auto arc : p)
    {
        if(arc->getType() == Arc::Swap)
        {
            Arc* arcToRemove = std::static_pointer_cast<MagnussonSwapArcUserData>(arc->getUserData())->getCutArc();
        }
    }
}

void Magnusson::removeSwapArcs()
{
    // TODO:
    for(auto a : swapArcs_)
    {
        // unregister it from source and target node
        Node *source = a->getSourceNode();
        Node *target = a->getTargetNode();
        source->removeOutArc(a.get());
        target->removeInArc(a.get());
    }
    swapArcs_.clear();
}

} // namespace dpct
