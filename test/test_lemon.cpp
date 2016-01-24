#define BOOST_TEST_MODULE test_lemon

#include <iostream>
#include <boost/test/unit_test.hpp>

#include <lemon/adaptors.h>
#include <lemon/bellman_ford.h>

#define private public
#include "graph.h"
#include "flowgraph.h"
#include "residualgraph.h"


using namespace dpct;

BOOST_AUTO_TEST_CASE(pure_lemon)
{
    typedef lemon::ListDigraph LGraph;
    typedef LGraph::Node Node;
    typedef LGraph::Arc Arc;
    typedef LGraph::ArcMap<double> DistMap;
    LGraph g;
    DistMap dist(g);

    Node s = g.addNode();
    Node t = g.addNode();

    Node n_1_1 = g.addNode();
    Node n_1_2 = g.addNode();
    Node n_2_1 = g.addNode();
    Node n_2_2 = g.addNode();
    Node n_2_3 = g.addNode();

    Node d_1_1 = g.addNode();
    Node d_1_2 = g.addNode();

    Arc app1 = g.addArc(s, n_1_1);
    dist[app1] = 0.0;
    Arc app2 = g.addArc(s, n_1_2);
    dist[app2] = 0.0;

    Arc div1 = g.addArc(s, d_1_1);
    dist[div1] = -4.0;
    Arc div2 = g.addArc(s, d_1_2);
    dist[div2] = -4.0;

    Arc move1 = g.addArc(n_1_1, n_2_1);
    dist[move1] = -4.0;
    Arc move2 = g.addArc(n_1_1, n_2_2);
    dist[move2] = -3.0;
    Arc move3 = g.addArc(n_1_2, n_2_2);
    dist[move3] = -1.0;
    Arc move4 = g.addArc(n_1_2, n_2_3);
    dist[move4] = -4.0;

    Arc child1 = g.addArc(d_1_1, n_2_1);
    dist[child1] = -4.0;
    Arc child2 = g.addArc(d_1_1, n_2_2);
    dist[child2] = -3.0;
    Arc child3 = g.addArc(d_1_2, n_2_2);
    dist[child3] = -1.0;
    Arc child4 = g.addArc(d_1_2, n_2_3);
    dist[child4] = -4.0;

    Arc dis1 = g.addArc(n_2_1, t);
    dist[dis1] = -2.0;
    Arc dis2 = g.addArc(n_2_2, t);
    dist[dis2] = -2.0;
    Arc dis3 = g.addArc(n_2_3, t);
    dist[dis3] = -4.0;

    // graph adapter to hide the two division arcs
    LGraph::ArcMap<bool> divisionArcEnabledMap(g);
    for(LGraph::ArcIt a(g); a != lemon::INVALID; ++a)
        divisionArcEnabledMap[a] = true;
    divisionArcEnabledMap[div1] = false;
    divisionArcEnabledMap[div2] = false;

    typedef lemon::FilterArcs<LGraph> FilteredLGraph;
    FilteredLGraph filteredG(g, divisionArcEnabledMap);

    // ------------------------------------------------
    // find shortest path
    typedef lemon::BellmanFord<FilteredLGraph, DistMap> BellmanFord;
    BellmanFord bf(filteredG, dist);
    bf.init();
    bf.addSource(s);

    if(bf.checkedStart())
    {
        std::cout << "\n******************************\n[BellmanFord]: found shortest path at distance " << bf.dist(t) << std::endl;
        
        std::cout << "Found path is: ";
        for(Node v = t; v != s; v=bf.predNode(v)) 
        {
            std::cout << filteredG.id(v) << "=" << g.id(v) << " <- ";
        }
        std::cout << filteredG.id(s) << "=" << g.id(s) << std::endl;
    }

    // ------------------------------------------------
    // build up capacities and residual graph
    LGraph::ArcMap<int> capacities(g);
    LGraph::ArcMap<int> flowMap(g);
    for(LGraph::ArcIt a(g); a != lemon::INVALID; ++a)
    {
        capacities[a] = 1;
        flowMap[a] = 0;
    }

    for(Arc a = bf.predArc(t); a != lemon::INVALID; a=bf.predArc(g.source(a)))
    {
        std::cout << "setting arc " << "(" << g.id(g.source(a)) << ", " << g.id(g.target(a)) << ") to contain 1 flow" << std::endl;
        flowMap[a] = 1;
    }

    // update filtered graph to contain the appropriate division now!
    divisionArcEnabledMap[div2] = true;
    divisionArcEnabledMap[child4] = false; // cannot use the same child as parent path
    filteredG = FilteredLGraph(g, divisionArcEnabledMap);

    typedef lemon::ResidualDigraph< FilteredLGraph, LGraph::ArcMap<int>, LGraph::ArcMap<int> > ResidualGraph;
    typedef ResidualGraph::ArcMap<double> ResidualDistMap;

    // ------------------------------------------------
    // find shortest path in residual graph and augment flow
    {
        ResidualGraph residualG(filteredG, capacities, flowMap);
        ResidualDistMap residualDist(residualG);
        for(ResidualGraph::ArcIt a(residualG); a != lemon::INVALID; ++a)
        {
            if(residualG.forward(a))
            {
                residualDist[a] = dist[lemon::findArc(g, residualG.source(a), residualG.target(a))];
            }
            else
            {
                residualDist[a] = -1.0 * dist[lemon::findArc(g, residualG.target(a), residualG.source(a))];
            }
        }

        std::cout << "Residual Graph has edges: " << std::endl;
        for(ResidualGraph::ArcIt a(residualG); a != lemon::INVALID; ++a)
        {
            std::cout << "(" << residualG.id(residualG.source(a)) << ", " << residualG.id(residualG.target(a)) << ") " 
            << (residualG.forward(a)?"forward":"backward") 
            << " cost: " << residualDist[a]
            << std::endl;
        }

        typedef lemon::BellmanFord<ResidualGraph, ResidualDistMap> ResidualBellmanFord;
        ResidualBellmanFord rbf(residualG, residualDist);
        rbf.init();
        rbf.addSource(s);

        if(rbf.checkedStart())
        {
            std::cout << "\n******************************\n[ResidualBellmanFord]: found shortest path at distance " << rbf.dist(t) << std::endl;
            
            std::cout << "Found path is: ";
            for(Node v = t; v != s; v=rbf.predNode(v)) 
            {
                std::cout << residualG.id(v) << "=" << g.id(v) << " <- ";
            }
            std::cout << residualG.id(s) << "=" << g.id(s) << std::endl;

            for(ResidualGraph::Arc a = rbf.predArc(t); a != lemon::INVALID; a=rbf.predArc(residualG.source(a)))
            {
                int delta = (residualG.forward(a) ? 1 : -1);
                flowMap[a] += delta;
                std::cout << "setting arc " << "(" << g.id(g.source(a)) << ", " << g.id(g.target(a))
                        << "), delta: " << delta 
                        << " new flow: " << flowMap[a]
                        << (residualG.forward(a) ? " forward" : " backward")
                        << std::endl;
            }
        }
        else
        {
            std::cout << "\n******************************\n[ResidualBellmanFord]: found negative weight directed cycle!" << std::endl;
            lemon::Path<ResidualGraph> path = rbf.negativeCycle();
            for(lemon::Path<ResidualGraph>::ArcIt it(path); it != lemon::INVALID; ++it)
            {
                std::cout << "(" << residualG.id(residualG.source(it)) << "=" << g.id(residualG.source(it)) << ", " << residualG.id(residualG.target(it)) << "=" << g.id(residualG.target(it)) << ") ";
            }
            std::cout << std::endl;
        }

        // found path that used division 2 of node 3, so disallow "unusing" the parent of the division before the division
        divisionArcEnabledMap[div2] = true;
        divisionArcEnabledMap[app2] = false;
        filteredG = FilteredLGraph(g, divisionArcEnabledMap);
    }
    // ------------------------------------------------
    // again, update flow, should find path s,n_1_1,n_2_1,t (= 0,2,4,1)
    {
        ResidualGraph residualG(filteredG, capacities, flowMap);
        ResidualDistMap residualDist(residualG);
        for(ResidualGraph::ArcIt a(residualG); a != lemon::INVALID; ++a)
        {
            if(residualG.forward(a))
            {
                residualDist[a] = dist[lemon::findArc(g, residualG.source(a), residualG.target(a))];
            }
            else
            {
                residualDist[a] = -1.0 * dist[lemon::findArc(g, residualG.target(a), residualG.source(a))];
            }
        }

        std::cout << "Residual Graph has edges: " << std::endl;
        for(ResidualGraph::ArcIt a(residualG); a != lemon::INVALID; ++a)
        {
            std::cout << "(" << residualG.id(residualG.source(a)) << ", " << residualG.id(residualG.target(a)) << ") " 
            << (residualG.forward(a)?"forward":"backward") 
            << " cost: " << residualDist[a]
            << std::endl;
        }

        typedef lemon::BellmanFord<ResidualGraph, ResidualDistMap> ResidualBellmanFord;
        ResidualBellmanFord rbf(residualG, residualDist);
        rbf.init();
        rbf.addSource(s);

        if(rbf.checkedStart())
        {
            std::cout << "\n******************************\n[ResidualBellmanFord]: found shortest path at distance " << rbf.dist(t) << std::endl;
            
            std::cout << "Found path is: ";
            for(Node v = t; v != s; v=rbf.predNode(v)) 
            {
                std::cout << residualG.id(v) << "=" << g.id(v) << " <- ";
            }
            std::cout << residualG.id(s) << "=" << g.id(s) << std::endl;

            for(ResidualGraph::Arc a = rbf.predArc(t); a != lemon::INVALID; a=rbf.predArc(residualG.source(a)))
            {
                int delta = (residualG.forward(a) ? 1 : -1);
                flowMap[a] += delta;
                std::cout << "setting arc " << "(" << g.id(g.source(a)) << ", " << g.id(g.target(a))
                        << "), delta: " << delta 
                        << " new flow: " << flowMap[a]
                        << (residualG.forward(a) ? " forward" : " backward")
                        << std::endl;
            }
        }
        else
        {
            std::cout << "\n******************************\n[ResidualBellmanFord]: found negative weight directed cycle!" << std::endl;
            lemon::Path<ResidualGraph> path = rbf.negativeCycle();
            for(lemon::Path<ResidualGraph>::ArcIt it(path); it != lemon::INVALID; ++it)
            {
                std::cout << "(" << residualG.id(residualG.source(it)) << "=" << g.id(residualG.source(it)) << ", " << residualG.id(residualG.target(it)) << "=" << g.id(residualG.target(it)) << ") ";
            }
            std::cout << std::endl;
        }

        // found path that now enables division 1
        divisionArcEnabledMap[div1] = true;
        divisionArcEnabledMap[child1] = false;
        filteredG = FilteredLGraph(g, divisionArcEnabledMap);
    }

    // ------------------------------------------------
    // again, update flow, should find cycle s, d_1_1, n_2_2, d_1_2, s
    {
        ResidualGraph residualG(filteredG, capacities, flowMap);
        ResidualDistMap residualDist(residualG);
        for(ResidualGraph::ArcIt a(residualG); a != lemon::INVALID; ++a)
        {
            if(residualG.forward(a))
            {
                residualDist[a] = dist[lemon::findArc(g, residualG.source(a), residualG.target(a))];
            }
            else
            {
                residualDist[a] = -1.0 * dist[lemon::findArc(g, residualG.target(a), residualG.source(a))];
            }
        }

        std::cout << "Residual Graph has edges: " << std::endl;
        for(ResidualGraph::ArcIt a(residualG); a != lemon::INVALID; ++a)
        {
            std::cout << "(" << residualG.id(residualG.source(a)) << ", " << residualG.id(residualG.target(a)) << ") " 
            << (residualG.forward(a)?"forward":"backward") 
            << " cost: " << residualDist[a]
            << std::endl;
        }

        typedef lemon::BellmanFord<ResidualGraph, ResidualDistMap> ResidualBellmanFord;
        ResidualBellmanFord rbf(residualG, residualDist);
        rbf.init();
        rbf.addSource(s);

        if(rbf.checkedStart())
        {
            if(!rbf.reached(t))
            {
                std::cout << ">>>>>> Finished!" << std::endl;
                return;
            }

            std::cout << "\n******************************\n[ResidualBellmanFord]: found shortest path at distance " << rbf.dist(t) << std::endl;
            
            std::cout << "Found path is: ";
            for(Node v = t; v != s; v=rbf.predNode(v)) 
            {
                std::cout << residualG.id(v) << "=" << g.id(v) << " <- ";
            }
            std::cout << residualG.id(s) << "=" << g.id(s) << std::endl;

            for(ResidualGraph::Arc a = rbf.predArc(t); a != lemon::INVALID; a=rbf.predArc(residualG.source(a)))
            {
                int delta = (residualG.forward(a) ? 1 : -1);
                flowMap[a] += delta;
                std::cout << "setting arc " << "(" << g.id(g.source(a)) << ", " << g.id(g.target(a))
                        << "), delta: " << delta 
                        << " new flow: " << flowMap[a]
                        << (residualG.forward(a) ? " forward" : " backward")
                        << std::endl;
            }
        }
        else
        {
            std::cout << "\n******************************\n[ResidualBellmanFord]: found negative weight directed cycle!" << std::endl;
            lemon::Path<ResidualGraph> path = rbf.negativeCycle();
            for(lemon::Path<ResidualGraph>::ArcIt it(path); it != lemon::INVALID; ++it)
            {
                int delta = (residualG.forward(it) ? 1 : -1);
                flowMap[lemon::findArc(g, residualG.target(it), residualG.source(it))] += delta;
                std::cout << "(" << residualG.id(residualG.source(it)) << "=" << g.id(residualG.source(it)) << ", " << residualG.id(residualG.target(it)) << "=" << g.id(residualG.target(it)) 
                        << "), delta: " << delta 
                        << " new flow: " << flowMap[lemon::findArc(g, residualG.target(it), residualG.source(it))]
                        << (residualG.forward(it) ? " forward" : " backward")
                        << std::endl;
            }
        }

        // flow has been redirected to use cheaper path
        // but we still have to re-enable the move from
        divisionArcEnabledMap[app2] = true;
        filteredG = FilteredLGraph(g, divisionArcEnabledMap);
    }

    // ------------------------------------------------
    // again, but there shouldn't be any paths left
    {
        ResidualGraph residualG(filteredG, capacities, flowMap);
        ResidualDistMap residualDist(residualG);
        for(ResidualGraph::ArcIt a(residualG); a != lemon::INVALID; ++a)
        {
            if(residualG.forward(a))
            {
                residualDist[a] = dist[lemon::findArc(g, residualG.source(a), residualG.target(a))];
            }
            else
            {
                residualDist[a] = -1.0 * dist[lemon::findArc(g, residualG.target(a), residualG.source(a))];
            }
        }

        std::cout << "Residual Graph has edges: " << std::endl;
        for(ResidualGraph::ArcIt a(residualG); a != lemon::INVALID; ++a)
        {
            std::cout << "(" << residualG.id(residualG.source(a)) << ", " << residualG.id(residualG.target(a)) << ") " 
            << (residualG.forward(a)?"forward":"backward") 
            << " cost: " << residualDist[a]
            << std::endl;
        }

        typedef lemon::BellmanFord<ResidualGraph, ResidualDistMap> ResidualBellmanFord;
        ResidualBellmanFord rbf(residualG, residualDist);
        rbf.init();
        rbf.addSource(s);

        if(rbf.checkedStart())
        {
            if(!rbf.reached(t))
            {
                std::cout << ">>>>>> Finished!" << std::endl;
                return;
            }

            std::cout << "\n******************************\n[ResidualBellmanFord]: found shortest path at distance " << rbf.dist(t) << std::endl;
            
            std::cout << "Found path is: ";
            for(Node v = t; v != s; v=rbf.predNode(v)) 
            {
                std::cout << residualG.id(v) << "=" << g.id(v) << " <- ";
            }
            std::cout << residualG.id(s) << "=" << g.id(s) << std::endl;

            for(ResidualGraph::Arc a = rbf.predArc(t); a != lemon::INVALID; a=rbf.predArc(residualG.source(a)))
            {
                int delta = (residualG.forward(a) ? 1 : -1);
                flowMap[a] += delta;
                std::cout << "setting arc " << "(" << g.id(g.source(a)) << ", " << g.id(g.target(a))
                        << "), delta: " << delta 
                        << " new flow: " << flowMap[a]
                        << (residualG.forward(a) ? " forward" : " backward")
                        << std::endl;
            }
        }
        else
        {
            std::cout << "\n******************************\n[ResidualBellmanFord]: found negative weight directed cycle!" << std::endl;
            lemon::Path<ResidualGraph> path = rbf.negativeCycle();
            for(lemon::Path<ResidualGraph>::ArcIt it(path); it != lemon::INVALID; ++it)
            {
                int delta = (residualG.forward(it) ? 1 : -1);
                std::cout << "(" << residualG.id(residualG.source(it)) << "=" << g.id(residualG.source(it)) << ", " << residualG.id(residualG.target(it)) << "=" << g.id(residualG.target(it)) << ") ";
                flowMap[lemon::findArc(g, residualG.source(it), residualG.target(it))] += delta;
            }
            std::cout << std::endl;
        }
    }
}

BOOST_AUTO_TEST_CASE( flowgraph_simple )
{
    FlowGraph g;
    typedef FlowGraph::FullNode Node;
    typedef FlowGraph::Arc Arc;

    Node n_1_1 = g.addNode({0.0});
    Node n_1_2 = g.addNode({0.0});
    Node n_1_3 = g.addNode({0.0});
    Node n_2_1 = g.addNode({0.0});
    Node n_2_2 = g.addNode({0.0});
    Node n_2_3 = g.addNode({0.0});

    FlowGraph::Node s = g.getSource();
    FlowGraph::Node t = g.getTarget();

    Arc app1 = g.addArc(s, n_1_1.u, {0.0});
    Arc app2 = g.addArc(s, n_1_2.u, {0.0});
    Arc app3 = g.addArc(s, n_2_1.u, {10.0});
    Arc app4 = g.addArc(s, n_2_2.u, {10.0});
    Arc app5 = g.addArc(s, n_2_3.u, {10.0});
    Arc app6 = g.addArc(s, n_1_3.u, {0.0});

    Arc move1 = g.addArc(n_1_1, n_2_1, {-4.0});
    Arc move2 = g.addArc(n_1_1, n_2_2, {-3.0});
    Arc move3 = g.addArc(n_1_2, n_2_2, {-1.0});
    Arc move4 = g.addArc(n_1_2, n_2_3, {-4.0});
    Arc move5 = g.addArc(n_1_3, n_2_3, {2.0});

    Arc dis1 = g.addArc(n_2_1.v, t, {-2.0});
    Arc dis2 = g.addArc(n_2_2.v, t, {-2.0});
    Arc dis3 = g.addArc(n_2_3.v, t, {-4.0});
    Arc dis4 = g.addArc(n_1_1.v, t, {10.0});
    Arc dis5 = g.addArc(n_1_2.v, t, {10.0});
    Arc dis6 = g.addArc(n_1_3.v, t, {-1.0});

    Arc div1 = g.allowMitosis(n_1_1, {-4.0});
    Arc div2 = g.allowMitosis(n_1_2, {-4.0});

    g.maxFlowMinCostTracking();

    BOOST_CHECK_EQUAL(g.getFlowMap()[app1], 1);
    BOOST_CHECK_EQUAL(g.getFlowMap()[app2], 1);
    BOOST_CHECK_EQUAL(g.getFlowMap()[app3], 0);
    BOOST_CHECK_EQUAL(g.getFlowMap()[app4], 0);
    BOOST_CHECK_EQUAL(g.getFlowMap()[app5], 0);
    BOOST_CHECK_EQUAL(g.getFlowMap()[app6], 1);

    BOOST_CHECK_EQUAL(g.getFlowMap()[dis1], 1);
    BOOST_CHECK_EQUAL(g.getFlowMap()[dis2], 1);
    BOOST_CHECK_EQUAL(g.getFlowMap()[dis3], 1);
    BOOST_CHECK_EQUAL(g.getFlowMap()[dis4], 0);
    BOOST_CHECK_EQUAL(g.getFlowMap()[dis5], 0);
    BOOST_CHECK_EQUAL(g.getFlowMap()[dis6], 1);

    BOOST_CHECK_EQUAL(g.getFlowMap()[div1], 1);
    BOOST_CHECK_EQUAL(g.getFlowMap()[div2], 0);

    BOOST_CHECK_EQUAL(g.getFlowMap()[move1], 1);
    BOOST_CHECK_EQUAL(g.getFlowMap()[move2], 1);
    BOOST_CHECK_EQUAL(g.getFlowMap()[move3], 0);
    BOOST_CHECK_EQUAL(g.getFlowMap()[move4], 1);
    BOOST_CHECK_EQUAL(g.getFlowMap()[move5], 0);
}

BOOST_AUTO_TEST_CASE( tokenizedbellmanford_have_tokens )
{
    FlowGraph g;
    typedef FlowGraph::FullNode Node;
    typedef FlowGraph::Arc Arc;

    Node n_1_1 = g.addNode({0.0});
    Node n_2_1 = g.addNode({0.0});
    Node n_2_2 = g.addNode({0.0});

    FlowGraph::Node s = g.getSource();
    FlowGraph::Node t = g.getTarget();

    Arc app1 = g.addArc(s, n_1_1.u, {0.0});
    
    Arc move1 = g.addArc(n_1_1, n_2_1, {-4.0});
    Arc move2 = g.addArc(n_1_1, n_2_2, {-3.0});
    
    Arc dis1 = g.addArc(n_2_1.v, t, {-2.0});
    Arc dis2 = g.addArc(n_2_2.v, t, {-2.0});
    
    const FlowGraph::Graph& baseGraph = g.getGraph();
    ResidualGraph rg(baseGraph);

    // only enable forward arcs, we just want a simple scenario to check whether token collection works
    for(FlowGraph::Graph::ArcIt a(baseGraph); a != lemon::INVALID; ++a)
    {
        rg.updateArc(a, ResidualGraph::Forward, g.getArcCost(a, 0), 1);
    }

    size_t tokenId = 12;
    rg.addProvidedToken(app1, ResidualGraph::Forward, tokenId);
    rg.addForbiddenToken(move1, ResidualGraph::Forward, tokenId);

    ResidualGraph::ShortestPathResult sp = rg.findShortestPath(s, t);
    BOOST_CHECK(sp.second < 0); // check that we found an augmenting path
    g.printPath(sp.first);
    
    // make sure we did not go along forbidden move1 arc
    for(auto arcFlowPair : sp.first)
    {
        BOOST_CHECK(arcFlowPair.first != move1);
    }

    // try again, this time without forbidding the provided token
    rg.removeForbiddenToken(move1, ResidualGraph::Forward, tokenId);
    sp = rg.findShortestPath(s, t);
    BOOST_CHECK(sp.second < 0); // check that we found an augmenting path
    g.printPath(sp.first);
    
    // make sure we did not go along move2 arc, because move1 is cheaper now
    for(auto arcFlowPair : sp.first)
    {
        BOOST_CHECK(arcFlowPair.first != move2);
    }
}