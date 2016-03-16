#ifndef MAGNUSSON_GRAPH_BUILDER
#define MAGNUSSON_GRAPH_BUILDER

#include "graphbuilder.h"
#include "graph.h"

namespace dpct
{
	
// ----------------------------------------------------------------------------------------
/**
 * @brief Implementation of building methods for magnusson's graph
 */
class MagnussonGraphBuilder : public GraphBuilder {
public:
	MagnussonGraphBuilder(Graph* graph):
		graph_(graph)
	{}

	/**
	 * @brief Magnusson performs score maximization, so we have to multiply our cost deltas by -1
	 */
	CostDeltaVector flipSign(const CostDeltaVector& vec)
	{
		CostDeltaVector newVec;
		for(auto a : vec)
			newVec.push_back(-1.0 * a);
		return newVec;
	}

	void addNode(
		size_t id,
		const CostDeltaVector& detectionCosts,
		const CostDeltaVector& detectionCostDeltas, 
		const CostDeltaVector& appearanceCostDeltas, 
		const CostDeltaVector& disappearanceCostDeltas,
		size_t targetIdx=0)
	{
		if(idToTimestepsMap_.find(id) == idToTimestepsMap_.end())
			throw std::runtime_error("Node timesteps must be set for Magnusson to work");
		
		size_t timestep = idToTimestepsMap_[id].second;
		Graph::NodePtr n = graph_->addNode(timestep, flipSign(detectionCosts), flipSign(appearanceCostDeltas), flipSign(disappearanceCostDeltas), false, false);
		idToGraphNodeMap_[id] = n;
		graphNodeToIdMap_[n] = id;
	}

	void addArc(size_t srcId, size_t destId, const CostDeltaVector& costDeltas)
	{
		if(idToGraphNodeMap_.find(srcId) == idToGraphNodeMap_.end())
			throw std::runtime_error("Trying to add link but source node is not present in map");
		if(idToGraphNodeMap_.find(destId) == idToGraphNodeMap_.end())
			throw std::runtime_error("Trying to add link but destination node is not present in map");
		Graph::ArcPtr a = graph_->addMoveArc(idToGraphNodeMap_[srcId], idToGraphNodeMap_[destId], flipSign(costDeltas));
		idTupleToGraphArcMap_[std::make_pair(srcId, destId)] = a;
	}

	void allowMitosis(size_t id, ValueType divisionCostDelta)
	{
		if(idToGraphNodeMap_.find(id) == idToGraphNodeMap_.end())
			throw std::runtime_error("Trying to add division but parent node is not present in map");
		Graph::NodePtr n = idToGraphNodeMap_[id];
		bool foundAny = false;
		n->visitOutArcs([&](Arc* a){
			if(a->getTargetNode() == nullptr)
				throw std::runtime_error("Found out arc pointing to nullptr!!!");
			if(a != n->getDisappearanceArc() && !graph_->isSpecialNode(a->getTargetNode()))
			{
				Graph::ArcPtr ap = graph_->allowMitosis(n.get(), a->getTargetNode(), -1.0 * divisionCostDelta);
				divisionArcs_.push_back(ap);
				foundAny = true;
			}
		});

		if(foundAny)
			idToGraphDivisionMap_[id] = n;
		else
			DEBUG_MSG("Could not add division arcs for node " << id);
	}

	NodeValueMap getNodeValues()
	{
		if(magnussonNodeValueMap_.size() == 0)
			throw std::runtime_error("No active nodes were found when exporting results!");

		NodeValueMap nodeValueMap;
		
		for(auto iter : idToGraphNodeMap_)
		{
			if(magnussonNodeValueMap_.find(iter.second.get()) != magnussonNodeValueMap_.end())
				nodeValueMap[iter.first] = magnussonNodeValueMap_[iter.second.get()];
		}

		return nodeValueMap;
	}

	ArcValueMap getArcValues()
	{
		ArcValueMap arcValueMap;
		
		for(auto iter : idTupleToGraphArcMap_)
		{
			if(magnussonArcValueMap_.find(iter.second.get()) != magnussonArcValueMap_.end())
				arcValueMap[iter.first] = magnussonArcValueMap_[iter.second.get()];
		}

		return arcValueMap;
	}

	DivisionValueMap getDivisionValues()
	{
		DivisionValueMap divisionValueMap;
		
		for(auto iter : idToGraphDivisionMap_)
		{
			if(magnussonDivisionValueMap_.find(iter.second.get()) != magnussonDivisionValueMap_.end())
				divisionValueMap[iter.first] = magnussonDivisionValueMap_[iter.second.get()];
		}

		return divisionValueMap;
	}

	AppearanceValueMap getAppearanceValues()
	{
		AppearanceValueMap appearanceValueMap;
		
		for(auto iter : idToGraphNodeMap_)
		{
			if(magnussonAppearanceValueMap_.find(iter.second.get()) != magnussonAppearanceValueMap_.end())
				appearanceValueMap[iter.first] = magnussonAppearanceValueMap_[iter.second.get()];
		}

		return appearanceValueMap;
	}

	DisappearanceValueMap getDisappearanceValues()
	{
		DisappearanceValueMap disappearanceValueMap;
		
		for(auto iter : idToGraphNodeMap_)
		{
			if(magnussonDisappearanceValueMap_.find(iter.second.get()) != magnussonDisappearanceValueMap_.end())
				disappearanceValueMap[iter.first] = magnussonDisappearanceValueMap_[iter.second.get()];
		}

		return disappearanceValueMap;
	}

	std::vector<FlowGraph::Path> translateSolution(const std::vector<TrackingAlgorithm::Path>& paths, FlowGraphBuilder& builder)
	{
		std::vector<FlowGraph::Path> flowPaths;
		flowPaths.reserve(paths.size());

	    for(const TrackingAlgorithm::Path& p : paths)
	    {
	    	FlowGraph::Path flowPath;
	        // a path starts at the dummy-source and goes to the dummy-sink. these arcs are of type dummy, and thus skipped
	        bool first_arc_on_path = true;
	        for(const Arc* a : p)
	        {
	            assert(a != nullptr);
	            assert(a->getSourceNode() != nullptr);
	            assert(a->getTargetNode() != nullptr);

	            switch(a->getType())
	            {
	                case Arc::Move:
	                {
	                    // send one cell through the nodes
	                    if(first_arc_on_path)
	                    {
	                        flowPath.push_back(std::make_pair(builder.getAppearanceArc(graphNodeToIdMap_[a->getSourceNode()->getSharedPtr()]), 1));
	                        flowPath.push_back(std::make_pair(builder.getNodeArc(graphNodeToIdMap_[a->getSourceNode()->getSharedPtr()]), 1));
	                        first_arc_on_path = false;
	                    }

	                    // set arc to active
	                    flowPath.push_back(std::make_pair(
	                    	builder.getMoveArc(std::make_pair(graphNodeToIdMap_[a->getSourceNode()->getSharedPtr()], 
	                    									  graphNodeToIdMap_[a->getTargetNode()->getSharedPtr()])), 
	                    	1));
	                    flowPath.push_back(std::make_pair(builder.getNodeArc(graphNodeToIdMap_[a->getTargetNode()->getSharedPtr()]), 1));
	                }
	                break;
	                case Arc::Appearance:
	                {
	                    // the node that appeared is set active here, so detections without further path are active as well
	                    flowPath.push_back(std::make_pair(builder.getAppearanceArc(graphNodeToIdMap_[a->getTargetNode()->getSharedPtr()]), 1));
                        flowPath.push_back(std::make_pair(builder.getNodeArc(graphNodeToIdMap_[a->getTargetNode()->getSharedPtr()]), 1));
	                    first_arc_on_path = false;
	                    
	                }
	                break;
	                case Arc::Disappearance:
	                {
	                    if(first_arc_on_path)
	                    {
	                    	flowPath.push_back(std::make_pair(builder.getAppearanceArc(graphNodeToIdMap_[a->getSourceNode()->getSharedPtr()]), 1));
	                        flowPath.push_back(std::make_pair(builder.getNodeArc(graphNodeToIdMap_[a->getSourceNode()->getSharedPtr()]), 1));
	                    }
	                    first_arc_on_path = false;
	                    flowPath.push_back(std::make_pair(builder.getDisappearanceArc(graphNodeToIdMap_[a->getSourceNode()->getSharedPtr()]), 1));
	                }
	                break;
	                case Arc::Division:
	                {
	                	assert(a->getObservedNode() != nullptr);
	                	auto flowArcs = builder.getDivisionArcs(graphNodeToIdMap_[a->getObservedNode()->getSharedPtr()], graphNodeToIdMap_[a->getTargetNode()->getSharedPtr()]);
	                	flowPath.push_back(std::make_pair(flowArcs.first, 1));
	                	flowPath.push_back(std::make_pair(flowArcs.second, 1));

	                    flowPath.push_back(std::make_pair(builder.getNodeArc(graphNodeToIdMap_[a->getTargetNode()->getSharedPtr()]), 1));
	                    first_arc_on_path = false;
	                }
	                break;
	                case Arc::Swap:
	                {
	                    throw std::runtime_error("Got a swap arc even though it should have been cleaned up!");
	                }
	                break;
	                case Arc::Dummy:
	                {
	                    // do nothing
	                } break;
	                default:
	                {
	                    throw std::runtime_error("Unkown arc type");
	                }
	                break;
	            } // switch
	        } // for all arcs
	        flowPaths.push_back(flowPath);
	    } // for all paths
	    return flowPaths;
	}

	void getSolutionFromPaths(const std::vector<TrackingAlgorithm::Path>& paths)
	{
		if(paths.size() == 0)
		{
			LOG_MSG("Got empty paths list, not extracting solution....");
			return;
		}

		magnussonNodeValueMap_.clear();
		magnussonArcValueMap_.clear();
		magnussonDivisionValueMap_.clear();
		magnussonAppearanceValueMap_.clear();
		magnussonDisappearanceValueMap_.clear();

		auto increase_object_count = [&](const Node* n){
			if(magnussonNodeValueMap_.find(n) == magnussonNodeValueMap_.end())
				magnussonNodeValueMap_[n] = 1;
			else
				magnussonNodeValueMap_[n] += 1;
		};

		auto activate_arc = [&](const Arc* a)
		{
			if(magnussonArcValueMap_.find(a) == magnussonArcValueMap_.end())
				magnussonArcValueMap_[a] = 1;
			else
				magnussonArcValueMap_[a] += 1;
		};

		auto activate_appearance = [&](const Node* n)
		{
			if(magnussonAppearanceValueMap_.find(n) == magnussonAppearanceValueMap_.end())
				magnussonAppearanceValueMap_[n] = 1;
			else
				magnussonAppearanceValueMap_[n] += 1;
		};

		auto activate_disappearance = [&](const Node* n)
		{
			if(magnussonDisappearanceValueMap_.find(n) == magnussonDisappearanceValueMap_.end())
				magnussonDisappearanceValueMap_[n] = 1;
			else
				magnussonDisappearanceValueMap_[n] += 1;
		};

		// fill solution vectors from the given paths
		// for each path, increment the number of cells the nodes and arcs along the path
	    for(const TrackingAlgorithm::Path& p : paths)
	    {
	        // a path starts at the dummy-source and goes to the dummy-sink. these arcs are of type dummy, and thus skipped
	        bool first_arc_on_path = true;
	        for(const Arc* a : p)
	        {
	            assert(a != nullptr);
	            assert(a->getSourceNode() != nullptr);
	            assert(a->getTargetNode() != nullptr);

	            switch(a->getType())
	            {
	                case Arc::Move:
	                {
	                    // send one cell through the nodes
	                    if(first_arc_on_path)
	                    {
	                        increase_object_count(a->getSourceNode());
	                        first_arc_on_path = false;
	                    }
	                    increase_object_count(a->getTargetNode());

	                    // set arc to active
	                    activate_arc(a);
	                }
	                break;
	                case Arc::Appearance:
	                {
	                    // the node that appeared is set active here, so detections without further path are active as well
	                    increase_object_count(a->getTargetNode());
	                    first_arc_on_path = false;
	                    activate_appearance(a->getTargetNode());
	                }
	                break;
	                case Arc::Disappearance:
	                {
	                    if(first_arc_on_path)
	                    {
	                        increase_object_count(a->getSourceNode());
	                    }
	                    first_arc_on_path = false;
	                    activate_disappearance(a->getSourceNode());
	                    // nothing to do, last node on path was already set active by previous move or appearance
	                }
	                break;
	                case Arc::Division:
	                {
	                	assert(a->getObservedNode() != nullptr);
	                	magnussonDivisionValueMap_[a->getObservedNode()] = true;

	                	// activate corresponding arc, which is not active in dpct!
	                    for(Node::ConstArcIt ai = a->getObservedNode()->getOutArcsBegin();
	                    	ai != a->getObservedNode()->getOutArcsEnd();
	                    	++ai)
	                    {
	                        if((*ai)->getTargetNode() == a->getTargetNode())
	                        {
	                            magnussonArcValueMap_[*ai] = 1;
	                        }
	                    }
	                    
	                    increase_object_count(a->getTargetNode());
	                    first_arc_on_path = false;
	                }
	                break;
	                case Arc::Swap:
	                {
	                    throw std::runtime_error("Got a swap arc even though it should have been cleaned up!");
	                }
	                break;
	                case Arc::Dummy:
	                {
	                    // do nothing
	                } break;
	                default:
	                {
	                    throw std::runtime_error("Unkown arc type");
	                }
	                break;
	            } // switch
	        } // for all arcs
	    } // for all paths
	}

private:
	/// pointer to magnusson's graph
	Graph* graph_;

	/// mapping from id to nodes
	std::map<size_t, Graph::NodePtr> idToGraphNodeMap_;
	std::map<Graph::NodePtr, size_t> graphNodeToIdMap_;

	/// mapping from id to division arc
	std::map<size_t, Graph::NodePtr> idToGraphDivisionMap_;

	/// mapping from tuple (id,id) to arc
	std::map<std::pair<size_t, size_t>, Graph::ArcPtr> idTupleToGraphArcMap_;

	/// list of all division arcs - shared ptr references so that the arc doesn't get deleted
	std::vector<Graph::ArcPtr> divisionArcs_;

	/// result maps that are filled when a set of paths is passed in
	std::map<const Node*, size_t> magnussonNodeValueMap_;
	std::map<const Arc*, size_t> magnussonArcValueMap_;
	std::map<const Node*, size_t> magnussonDivisionValueMap_;
	std::map<const Node*, size_t> magnussonAppearanceValueMap_;
	std::map<const Node*, size_t> magnussonDisappearanceValueMap_;
};
}

#endif