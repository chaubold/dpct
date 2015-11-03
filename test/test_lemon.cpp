#define BOOST_TEST_MODULE test_lemon

#include <iostream>
#include <boost/test/unit_test.hpp>
#include "graph.h"
#include "lemongraph.h"

#include <lemon/adaptors.h>
#include <lemon/bellman_ford.h>

using namespace dpct;

BOOST_AUTO_TEST_CASE(build_lemongraph)
{
    Graph::Configuration config(true, true, true);
    Graph g(config);
    BOOST_CHECK_EQUAL(g.getNumArcs(), 0);
    BOOST_CHECK_EQUAL(g.getNumNodes(), 0);

    double appearanceScore = -200;
    double disappearanceScore = -200;
    double divisionScore = -200;

    // -----------------------------------------------------
    // Timestep 1
    Graph::NodePtr n_1_1 = g.addNode(0, {3, -1}, appearanceScore, disappearanceScore,
                                        true, false, std::make_shared<NameData>("Timestep 1: Node 1"));

    Graph::NodePtr n_1_2 = g.addNode(0, {2, 5, -2}, appearanceScore, disappearanceScore,
                                        true, false, std::make_shared<NameData>("Timestep 1: Node 2"));

    Graph::NodePtr n_1_3 = g.addNode(0, {3, -5}, appearanceScore, disappearanceScore,
                                        true, false, std::make_shared<NameData>("Timestep 1: Node 3"));

    BOOST_CHECK_EQUAL(g.getNumArcs(), 6);
    BOOST_CHECK_EQUAL(g.getNumNodes(), 3);

    // -----------------------------------------------------
    // Timestep 2
    Graph::NodePtr n_2_1 = g.addNode(1, {4, -2}, appearanceScore, disappearanceScore,
                                        false, false, std::make_shared<NameData>("Timestep 2: Node 1"));

    Graph::NodePtr n_2_2 = g.addNode(1, {2, -2}, appearanceScore, disappearanceScore,
                                        false, false, std::make_shared<NameData>("Timestep 2: Node 2"));

    Graph::NodePtr n_2_3 = g.addNode(1, {2, -4}, appearanceScore, disappearanceScore,
                                        false, false, std::make_shared<NameData>("Timestep 2: Node 3"));

    Graph::NodePtr n_2_4 = g.addNode(1, {2,-2}, appearanceScore, disappearanceScore,
                                        false, false, std::make_shared<NameData>("Timestep 2: Node 4"));

    BOOST_CHECK_EQUAL(g.getNumArcs(), 14);
    BOOST_CHECK_EQUAL(g.getNumNodes(), 7);

    g.addMoveArc(n_1_1, n_2_1, 0.0, std::make_shared<NameData>("Arc 1"));
    g.addMoveArc(n_1_2, n_2_2, 0.0);
    g.addMoveArc(n_1_2, n_2_3, 0.0);
    g.addMoveArc(n_1_3, n_2_4, 0.0);

    BOOST_CHECK_EQUAL(g.getNumArcs(), 18);
    BOOST_CHECK_EQUAL(g.getNumNodes(), 7);

    // -----------------------------------------------------
    // Timestep 3
    Graph::NodePtr n_3_1 = g.addNode(2, {3, 2}, appearanceScore, disappearanceScore,
                                        false, false);

    Graph::NodePtr n_3_2 = g.addNode(2, {2, 0}, appearanceScore, disappearanceScore,
                                        false, false);

    Graph::NodePtr n_3_3 = g.addNode(2, {3, -3}, appearanceScore, disappearanceScore,
                                        false, false);

    BOOST_CHECK_EQUAL(g.getNumArcs(), 24);
    BOOST_CHECK_EQUAL(g.getNumNodes(), 10);

    g.addMoveArc(n_2_1, n_3_1, 0.0);
    g.addMoveArc(n_2_2, n_3_2, 0.0);
    g.addMoveArc(n_2_3, n_3_3, 0.0);
    BOOST_CHECK_EQUAL(g.getNumArcs(), 27);

    // -----------------------------------------------------
    // Timestep 4
    Graph::NodePtr n_4_1 = g.addNode(3, {4, -1}, appearanceScore, disappearanceScore,
                                        false, true);

    Graph::NodePtr n_4_2 = g.addNode(3, {2, -1}, appearanceScore, disappearanceScore,
                                        false, true);

    Graph::NodePtr n_4_3 = g.addNode(3, {2, -6}, appearanceScore, disappearanceScore,
                                        false, true);

    Graph::NodePtr n_4_4 = g.addNode(3, {4, -2}, appearanceScore, disappearanceScore,
                                        false, true);

    BOOST_CHECK_EQUAL(g.getNumArcs(), 35); // doesn't add disappearance moves at sink
    BOOST_CHECK_EQUAL(g.getNumNodes(), 14);

    g.addMoveArc(n_3_1, n_4_1, 0.0);
    g.addMoveArc(n_3_1, n_4_2, 0.0);
    g.addMoveArc(n_3_2, n_4_2, 0.0);
    g.addMoveArc(n_3_2, n_4_3, 0.0);
    g.addMoveArc(n_3_3, n_4_3, 0.0);
    g.addMoveArc(n_3_3, n_4_4, 0.0);
    BOOST_CHECK_EQUAL(g.getNumArcs(), 41);

    g.allowMitosis(n_3_2, n_4_3, -1);
    g.allowMitosis(n_3_3, n_4_3, 2);

    BOOST_CHECK_EQUAL(g.getNumArcs(), 43);
    // -----------------------------------------------------
    LemonGraph lg(&g);
    BOOST_CHECK_EQUAL(lg.getNumArcs(), g.getNumArcs());
    BOOST_CHECK_EQUAL(lg.getNumNodes(), g.getNumNodes() + 2); // + source and sink
}

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