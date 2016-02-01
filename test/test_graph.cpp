#define BOOST_TEST_MODULE test_graph

#include <iostream>
#include <boost/test/unit_test.hpp>
#include "graph.h"

using namespace dpct;

BOOST_AUTO_TEST_CASE(node_arcs)
{
    Node n1(std::vector<double>(), nullptr);
    Node n2(std::vector<double>(), nullptr);
    BOOST_CHECK_EQUAL(n1.getNumInArcs(),  0);
    BOOST_CHECK_EQUAL(n1.getNumOutArcs(), 0);

    BOOST_CHECK_EQUAL(n2.getNumInArcs(),  0);
    BOOST_CHECK_EQUAL(n2.getNumOutArcs(), 0);

    Arc a(&n1, &n2, Arc::Move, {0.0});

    BOOST_CHECK_EQUAL(n1.getNumInArcs(),  0);
    BOOST_CHECK_EQUAL(n1.getNumOutArcs(), 1);

    BOOST_CHECK_EQUAL(n2.getNumInArcs(),  1);
    BOOST_CHECK_EQUAL(n2.getNumOutArcs(), 0);

    a.getSourceNode()->removeOutArc(&a);
    BOOST_CHECK_EQUAL(n1.getNumInArcs(),  0);
    BOOST_CHECK_EQUAL(n1.getNumOutArcs(), 0);

    BOOST_CHECK_EQUAL(n2.getNumInArcs(),  1);
    BOOST_CHECK_EQUAL(n2.getNumOutArcs(), 0);

    a.getTargetNode()->removeInArc(&a);
    BOOST_CHECK_EQUAL(n1.getNumInArcs(),  0);
    BOOST_CHECK_EQUAL(n1.getNumOutArcs(), 0);

    BOOST_CHECK_EQUAL(n2.getNumInArcs(),  0);
    BOOST_CHECK_EQUAL(n2.getNumOutArcs(), 0);
}

BOOST_AUTO_TEST_CASE(build_graph)
{
    Graph::Configuration config(true, true, true);
    Graph g(config);
    BOOST_CHECK_EQUAL(g.getNumArcs(), 0);
    BOOST_CHECK_EQUAL(g.getNumNodes(), 0);

    std::vector<double> appearanceScore(2, -200);
    std::vector<double> disappearanceScore(2, -200);
    std::vector<double> divisionScore(2, -200);

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

    g.addMoveArc(n_1_1, n_2_1, {0.0}, std::make_shared<NameData>("Arc 1"));
    g.addMoveArc(n_1_2, n_2_2, {0.0});
    g.addMoveArc(n_1_2, n_2_3, {0.0});
    g.addMoveArc(n_1_3, n_2_4, {0.0});

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

    g.addMoveArc(n_2_1, n_3_1, {0.0});
    g.addMoveArc(n_2_2, n_3_2, {0.0});
    g.addMoveArc(n_2_3, n_3_3, {0.0});
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

    g.addMoveArc(n_3_1, n_4_1, {0.0});
    g.addMoveArc(n_3_1, n_4_2, {0.0});
    Graph::ArcPtr m1 = g.addMoveArc(n_3_2, n_4_2, {0.0});
    Graph::ArcPtr m2 = g.addMoveArc(n_3_2, n_4_3, {0.0});
    g.addMoveArc(n_3_3, n_4_3, {0.0});
    g.addMoveArc(n_3_3, n_4_4, {0.0});
    BOOST_CHECK_EQUAL(g.getNumArcs(), 41);

    Graph::ArcPtr d1 = g.allowMitosis(n_3_2, n_4_3, -1);
    Graph::ArcPtr d2 = g.allowMitosis(n_3_3, n_4_3, 2);

    BOOST_CHECK_EQUAL(g.getNumArcs(), 43);
    BOOST_CHECK_EQUAL(d1->isEnabled(), false);
    BOOST_CHECK_EQUAL(d2->isEnabled(), false);
    BOOST_CHECK_EQUAL(m1->isEnabled(), true);

    // add a cell to a division's mother and activate one of its out arc to enable a division
    BOOST_CHECK_EQUAL(n_3_2->getCellCount(), 0);
    n_3_2->increaseCellCount();
    BOOST_CHECK_EQUAL(n_3_2->getCellCount(), 1);
    BOOST_CHECK_EQUAL(m1->getUseCount(), 0);
    m1->markUsed();
    BOOST_CHECK_EQUAL(m1->getUseCount(), 1);
    // this should enable the division arc
    BOOST_CHECK_EQUAL(d1->isEnabled(), true);
    BOOST_CHECK_EQUAL(m2->isEnabled(), true);
    // now let's use the division arc and see whether the other transitions become invalid
    d1->markUsed();
    BOOST_CHECK_EQUAL(m1->isEnabled(), false);
    BOOST_CHECK_EQUAL(m2->isEnabled(), false);
    BOOST_CHECK_EQUAL(d1->isEnabled(), false);

    // unuse again
    d1->markUsed(false);
    BOOST_CHECK_EQUAL(m1->isEnabled(), true);
    BOOST_CHECK_EQUAL(m2->isEnabled(), true);
    BOOST_CHECK_EQUAL(d1->isEnabled(), true);

    // use other out arc and add another cell, then the division should be disabled
    n_3_2->increaseCellCount();
    m2->markUsed();
    BOOST_CHECK_EQUAL(d1->isEnabled(), false);
}

BOOST_AUTO_TEST_CASE(node_active_arc_counts)
{
    Node n1(std::vector<double>(), nullptr);
    Node n2(std::vector<double>(), nullptr);

    Arc a(&n1, &n2, Arc::Move, {0.0});
    BOOST_CHECK_EQUAL(n1.getNumActiveDivisions(), 0);
    BOOST_CHECK_EQUAL(n1.getMoveInArcUsedSum(), 0);
    BOOST_CHECK_EQUAL(n1.getMoveOutArcUsedSum(), 0);

    a.markUsed();
    BOOST_CHECK_EQUAL(n1.getNumActiveDivisions(), 0);
    BOOST_CHECK_EQUAL(n1.getMoveInArcUsedSum(), 0);
    BOOST_CHECK_EQUAL(n1.getMoveOutArcUsedSum(), 1);

    BOOST_CHECK_EQUAL(n2.getNumActiveDivisions(), 0);
    BOOST_CHECK_EQUAL(n2.getMoveInArcUsedSum(), 1);
    BOOST_CHECK_EQUAL(n2.getMoveOutArcUsedSum(), 0);

    a.markUsed();
    BOOST_CHECK_EQUAL(n1.getNumActiveDivisions(), 0);
    BOOST_CHECK_EQUAL(n1.getMoveInArcUsedSum(), 0);
    BOOST_CHECK_EQUAL(n1.getMoveOutArcUsedSum(), 2);

    BOOST_CHECK_EQUAL(n2.getNumActiveDivisions(), 0);
    BOOST_CHECK_EQUAL(n2.getMoveInArcUsedSum(), 2);
    BOOST_CHECK_EQUAL(n2.getMoveOutArcUsedSum(), 0);

    a.markUsed(false);
    BOOST_CHECK_EQUAL(n1.getNumActiveDivisions(), 0);
    BOOST_CHECK_EQUAL(n1.getMoveInArcUsedSum(), 0);
    BOOST_CHECK_EQUAL(n1.getMoveOutArcUsedSum(), 1);

    BOOST_CHECK_EQUAL(n2.getNumActiveDivisions(), 0);
    BOOST_CHECK_EQUAL(n2.getMoveInArcUsedSum(), 1);
    BOOST_CHECK_EQUAL(n2.getMoveOutArcUsedSum(), 0);

    a.markUsed(false);
    BOOST_CHECK_EQUAL(n1.getNumActiveDivisions(), 0);
    BOOST_CHECK_EQUAL(n1.getMoveInArcUsedSum(), 0);
    BOOST_CHECK_EQUAL(n1.getMoveOutArcUsedSum(), 0);

    BOOST_CHECK_EQUAL(n2.getNumActiveDivisions(), 0);
    BOOST_CHECK_EQUAL(n2.getMoveInArcUsedSum(), 0);
    BOOST_CHECK_EQUAL(n2.getMoveOutArcUsedSum(), 0);
}
