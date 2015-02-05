#include <functional>
#include <algorithm>
#include <assert.h>
#include <sstream>

#include "magnusson.h"
#include "trackingalgorithm.h"
#include "graph.h"
#include "log.h"

using std::placeholders::_1;

namespace dpct
{

Magnusson::Magnusson(Graph* graph, bool withSwap, bool usedArcsScoreZero):
    TrackingAlgorithm(graph),
    withSwap_(withSwap),
    usedArcsScoreZero_(usedArcsScoreZero)
{}

double Magnusson::track(std::vector<TrackingAlgorithm::Path>& paths)
{
    tic();
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
        DEBUG_MSG("Current best path of length " << p.size() << " has score: " << p.back()->getCurrentScore());
        printPath(p);
        scoreDelta = p.back()->getCurrentScore();

        // only continue if this path adds to the overall score
        if(scoreDelta < 0)
        {
            DEBUG_MSG("Path has negative reward, stopping here with a total number of " << paths.size() << " cells added");
            break;
        }

        // insert swap arcs
        if(withSwap_)
        {
            cleanUpUsedSwapArcs(p, paths);

            // make sure no swap arcs are left in the paths, otherwise we'll access free'd memory lateron
            for(Path& path : paths)
            { 
                for(Node::ArcIt p_it = path.begin(); p_it != path.end(); ++p_it)
                {
                    assert((*p_it)->getType() != Arc::Swap);
                }
            }

            // first remove all swap arcs, then add new ones!
            removeSwapArcs();
            for(Path &path : paths)
                insertSwapArcsForNewUsedPath(path);

            insertSwapArcsForNewUsedPath(p);
        }

        // update scores along that path and all nodes that go away from there
        // Node* firstPathNode = p.front()->getTargetNode();
        // if(firstPathNode->getUserData())
        // {
        //     DEBUG_MSG("Beginning update at node: " << *(std::static_pointer_cast<NameData>(firstPathNode->getUserData())));
        // }
        // else
        // {
        //     DEBUG_MSG("Beginning update at node " << firstPathNode);
        // }
        // breadthFirstSearchVisitor(firstPathNode, std::bind(&Magnusson::updateNode, this, _1));

        // update scores from timestep 0 to the end
        for(size_t t = 0; t < graph_->getNumTimesteps(); ++t)
        {
            graph_->visitNodesInTimestep(t, std::bind(&Magnusson::updateNode, this, _1));
        }
        graph_->visitSpecialNodes(std::bind(&Magnusson::updateNode, this, _1));

        // // clean up used arcs
        // if(withSwap_)
        // {
        //     cleanUpUsedSwapArcs(p, paths);

        //     // first remove all swap arcs, then add new ones!
        //     removeSwapArcs();
        //     for(Path &path : paths)
        //         insertSwapArcsForNewUsedPath(path);
        // }

        // add path to solution
        paths.push_back(p);
        score += scoreDelta;
        std::cout << "Added path with score " << scoreDelta << std::endl;
    };

    // done
    removeSwapArcs();
    toc();

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
        DEBUG_MSG("Increasing cell count of node " << *(std::static_pointer_cast<NameData>(n->getUserData())) << " = " << n);
    }
    else
    {
        DEBUG_MSG("Increasing cell count of node " << n);
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

        if(usedArcsScoreZero_ && bestArc->getType() == Arc::Move)
            bestArc->markUsed();
		p.push_back(bestArc);
		current = bestArc->getSourceNode();
	}

    nodeVisitor(current);
	std::reverse(p.begin(), p.end());
}

std::ostream& operator<<(std::ostream& s, Arc* arc)
{
    s << "(";
    if(arc->getSourceNode()->getUserData())
        s << arc->getSourceNode()->getUserData();
    else
        s << arc->getSourceNode();
    s << " --> ";

    if(arc->getTargetNode()->getUserData())
        s << arc->getTargetNode()->getUserData();
    else
        s << arc->getTargetNode();
    s << ")";
    return s;
}

void Magnusson::insertSwapArcsForNewUsedPath(TrackingAlgorithm::Path &p)
{
    /* Swap Arc Insertion: walk along path
     * - find alternative incoming arcs of the traversed node, where there are also multiple outgoing arcs of the previous node
     * - insert swap arc between any alternative pair of nodes connected to the path by in / out arcs.
    */
    for(Path::iterator it = p.begin(); it != p.end(); ++it)
    {
        // FIXME: sometimes points to a free'd memory location
        if((*it)->getType() != Arc::Move) // only swap Moves for now
            continue;

        Node *source = (*it)->getSourceNode();
        Node *target = (*it)->getTargetNode();

        for(Node::ArcIt outIt = source->getOutArcsBegin(); outIt != source->getOutArcsEnd(); ++outIt)
        {
            Node *alternativeTarget = (*outIt)->getTargetNode();
            if(alternativeTarget == target || graph_->isSpecialNode(alternativeTarget) || (*outIt)->getType() == Arc::Swap)
                continue;

            for(Node::ArcIt inIt = target->getInArcsBegin(); inIt != target->getInArcsEnd(); ++inIt)
            {
                Node *alternativeSource = (*inIt)->getSourceNode();
                if(alternativeSource == source || graph_->isSpecialNode(alternativeSource) || (*inIt)->getType() == Arc::Swap)
                    continue;

                // found a candidate
                double score = (*outIt)->getScoreDelta() + (*inIt)->getScoreDelta() - (*it)->getScoreDelta();
                // if there is an arc connecting the two alternative nodes, subtract its score delta as well
                for(Node::ArcIt alternativeOutIt = alternativeSource->getOutArcsBegin(); alternativeOutIt != alternativeSource->getOutArcsEnd(); ++alternativeOutIt)
                {
                    if( (*alternativeOutIt)->getTargetNode() == alternativeTarget
                            && (*alternativeOutIt)->getType() == Arc::Move) // only swap Moves for now
                            // && (*alternativeOutIt)->getType() != Arc::Swap) // is that needed? -> shouldn't be, could be "swapped" twice
                        score -= (*alternativeOutIt)->getScoreDelta();
                }

                // the swap arc does not depend on other nodes being part of a path,
                // as this algorithm never removes cells and thus the previously populated nodes can be used in swaps.
                // BUT: it needs to store a reference to the arc that it would cut, and the cleanup action is performed in cleanUpUsedSwapArcs()
                Arc* arc = new Arc(alternativeSource,
                                          alternativeTarget,
                                          Arc::Swap,
                                          score,
                                          nullptr,
                                          std::make_shared<MagnussonSwapArcUserData>(*it, *inIt, *outIt)
                                          );
                (*it)->registerObserverArc(arc);

#ifdef DEBUG_LOG
                std::stringstream debugString;
                debugString << "!!! Adding SWAP ARC between ";
                if(alternativeSource->getUserData())
                    debugString << alternativeSource->getUserData();
                else
                    debugString << alternativeSource;
                debugString << " and ";

                if(alternativeTarget->getUserData())
                    debugString << alternativeTarget->getUserData();
                else
                    debugString << alternativeTarget;

                debugString << " with score " << score;
                DEBUG_MSG(debugString.str());
                DEBUG_MSG("\twith replacement arcs: in=" << *inIt << " and out=" << *outIt);
                DEBUG_MSG("\tdeletes arc: " << *it);
#endif

                swapArcs_.push_back(arc);
            }
        }
    }
}

void Magnusson::cleanUpUsedSwapArcs(TrackingAlgorithm::Path &p, std::vector<Path>& paths)
{
    // if a swap arc was used, we can find the path that was affected by this and create the two paths after swapping
    bool foundSwapArc = true;
    while(foundSwapArc)
    {
        foundSwapArc = false;
        for(std::vector<Arc*>::reverse_iterator p_it = p.rbegin(); p_it != p.rend(); ++p_it)
        {
            // FIXME: sometimes points to free'd memory location
            Arc* arc = *p_it;
            if(arc->getType() == Arc::Swap)
            {
                assert(arc->getUserData());
                Arc* arcToRemove = std::static_pointer_cast<MagnussonSwapArcUserData>(arc->getUserData())->getCutArc();
                Arc* replacementP = std::static_pointer_cast<MagnussonSwapArcUserData>(arc->getUserData())->getReplacementAArc();
                Arc* replacementPath = std::static_pointer_cast<MagnussonSwapArcUserData>(arc->getUserData())->getReplacementBArc();
                assert(arcToRemove != nullptr);
                assert(replacementP != nullptr);
                assert(replacementPath != nullptr);

                LOG_MSG("Trying to remove swap arc between " << arc->getSourceNode()->getUserData()->toString() << " and " << arc->getTargetNode()->getUserData()->toString());

                for(Path& path : paths)
                {
                    for(Node::ArcIt path_it = path.begin(); path_it != path.end(); ++path_it)
                    {
                        Arc* a = *path_it;

                        if(a == arcToRemove)
                        {
                            // found candidate. store part of original path that will be used by new path
                            Path temp(path_it+1, path.end());

                            for(Node::ArcIt temp_it = temp.begin(); temp_it != temp.end(); ++temp_it)
                            {
                                assert((*temp_it)->getType() != Arc::Swap);
                            }

                            // update original path by appending the replacement arc and the remainder of path P
                            path.erase(path_it, path.end());
                            path.push_back(replacementPath);
                            path.insert(path.end(), p_it.base(), p.end());

                            // update path p
                            p.erase(p_it.base()-1, p.end());
                            p.push_back(replacementP);
                            p.insert(p.end(), temp.begin(), temp.end());

                            if(usedArcsScoreZero_)
                            {
                                replacementPath->markUsed();
                                replacementP->markUsed();
                                // "unuse" the arc that was previously used
                                arcToRemove->markUsed(false);
                            }

                            // remove all swap arcs from graph that were referencing the edge, including "arc". TODO: what to do if edge was used several times?
                            // LOG_MSG("updated two paths. removed swap arc between " << arc->getSourceNode()->getUserData()->toString() << " and " << arc->getTargetNode()->getUserData()->toString());
                            // arcToRemove->notifyObserverArcs(std::bind(&Magnusson::removeSwapArc, this, _1));

                            foundSwapArc = true;
                            break;
                        }
                    }

                    if(foundSwapArc)
                        break;
                }

                assert(foundSwapArc);
                break;
            }
        }
    }

    for(Node::ArcIt p_it = p.begin(); p_it != p.end(); ++p_it)
    {
        assert((*p_it)->getType() != Arc::Swap);
    }
}

void Magnusson::removeSwapArc(Arc* arc)
{
    assert(arc->getType() == Arc::Swap);

    removeArc(arc);
    std::vector<Arc*>::iterator it = std::find(swapArcs_.begin(), swapArcs_.end(), arc);
    if(it != swapArcs_.end())
        swapArcs_.erase(it);
    else
        throw std::runtime_error("Did not find arc to remove!");
}

void Magnusson::removeArc(Arc* a)
{
    // unregister it from source and target node
    Node *source = a->getSourceNode();
    Node *target = a->getTargetNode();
    source->removeOutArc(a);
    target->removeInArc(a);
}

void Magnusson::removeSwapArcs()
{
    for(Arc* a : swapArcs_)
    {
        removeArc(a);
        delete a;
    }
    swapArcs_.clear();
}

} // namespace dpct
