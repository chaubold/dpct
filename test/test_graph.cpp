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

    Arc a(&n1, &n2, Arc::Move, 0.0);

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
    BOOST_CHECK_EQUAL(g.getNumArcs(), 3);
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

    BOOST_CHECK_EQUAL(g.getNumArcs(), 9);
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

    BOOST_CHECK_EQUAL(g.getNumArcs(), 17);
    BOOST_CHECK_EQUAL(g.getNumNodes(), 7);

    g.addMoveArc(n_1_1, n_2_1, 0.0, std::make_shared<NameData>("Arc 1"));
    g.addMoveArc(n_1_2, n_2_2, 0.0);
    g.addMoveArc(n_1_2, n_2_3, 0.0);
    g.addMoveArc(n_1_3, n_2_4, 0.0);

    BOOST_CHECK_EQUAL(g.getNumArcs(), 21);
    BOOST_CHECK_EQUAL(g.getNumNodes(), 7);

    // -----------------------------------------------------
    // Timestep 3
    Graph::NodePtr n_3_1 = g.addNode(2, {3, 2}, appearanceScore, disappearanceScore,
                                        false, false);

    Graph::NodePtr n_3_2 = g.addNode(2, {2, 0}, appearanceScore, disappearanceScore,
                                        false, false);

    Graph::NodePtr n_3_3 = g.addNode(2, {3, -3}, appearanceScore, disappearanceScore,
                                        false, false);

    BOOST_CHECK_EQUAL(g.getNumArcs(), 27);
    BOOST_CHECK_EQUAL(g.getNumNodes(), 10);

    g.addMoveArc(n_2_1, n_3_1, 0.0);
    g.addMoveArc(n_2_2, n_3_2, 0.0);
    g.addMoveArc(n_2_3, n_3_3, 0.0);
    BOOST_CHECK_EQUAL(g.getNumArcs(), 30);

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

    BOOST_CHECK_EQUAL(g.getNumArcs(), 38); // doesn't add disappearance moves at sink
    BOOST_CHECK_EQUAL(g.getNumNodes(), 14);

    g.addMoveArc(n_3_1, n_4_1, 0.0);
    g.addMoveArc(n_3_1, n_4_2, 0.0);
    g.addMoveArc(n_3_2, n_4_2, 0.0);
    g.addMoveArc(n_3_2, n_4_3, 0.0);
    g.addMoveArc(n_3_3, n_4_3, 0.0);
    g.addMoveArc(n_3_3, n_4_4, 0.0);
    BOOST_CHECK_EQUAL(g.getNumArcs(), 44);

    g.allowMitosis(n_3_2, n_4_3, -1);
    g.allowMitosis(n_3_3, n_4_3, 2);

    BOOST_CHECK_EQUAL(g.getNumArcs(), 46);
}

BOOST_AUTO_TEST_CASE(copy_graph)
{
    Graph::Configuration config(true, true, true);
    Graph g(config);
    BOOST_CHECK_EQUAL(g.getNumArcs(), 3);
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

    g.addMoveArc(n_1_1, n_2_1, 0.0, std::make_shared<NameData>("Arc 1"));
    g.addMoveArc(n_1_2, n_2_2, 0.0);
    g.addMoveArc(n_1_2, n_2_3, 0.0);
    g.addMoveArc(n_1_3, n_2_4, 0.0);

    // -----------------------------------------------------
    // Timestep 3
    Graph::NodePtr n_3_1 = g.addNode(2, {3, 2}, appearanceScore, disappearanceScore,
                                        false, false);

    Graph::NodePtr n_3_2 = g.addNode(2, {2, 0}, appearanceScore, disappearanceScore,
                                        false, false);

    Graph::NodePtr n_3_3 = g.addNode(2, {3, -3}, appearanceScore, disappearanceScore,
                                        false, false);

    g.addMoveArc(n_2_1, n_3_1, 0.0);
    g.addMoveArc(n_2_2, n_3_2, 0.0);
    g.addMoveArc(n_2_3, n_3_3, 0.0);

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


    g.addMoveArc(n_3_1, n_4_1, 0.0);
    g.addMoveArc(n_3_1, n_4_2, 0.0);
    g.addMoveArc(n_3_2, n_4_2, 0.0);
    g.addMoveArc(n_3_2, n_4_3, 0.0);
    g.addMoveArc(n_3_3, n_4_3, 0.0);
    g.addMoveArc(n_3_3, n_4_4, 0.0);

    g.allowMitosis(n_3_2, n_4_3, -1);
    g.allowMitosis(n_3_3, n_4_3, 2);

    Graph g2(g);
    BOOST_CHECK_EQUAL(g2.getNumArcs(), g.getNumArcs());
    BOOST_CHECK_EQUAL(g2.getNumNodes(), g.getNumNodes());
    BOOST_CHECK_EQUAL(g2.getNumTimesteps(), g.getNumTimesteps());
    BOOST_CHECK_EQUAL(std::static_pointer_cast<NodeOriginData>(g2.getSinkNode().getUserData())->getOrigin()[0], &g.getSinkNode());
    BOOST_CHECK_EQUAL(std::static_pointer_cast<NodeOriginData>(g2.getSourceNode().getUserData())->getOrigin()[0], &g.getSourceNode());
    BOOST_CHECK_EQUAL(g2.getConfig().withAppearance, g.getConfig().withAppearance);
    BOOST_CHECK_EQUAL(g2.getConfig().withDisappearance, g.getConfig().withDisappearance);
    BOOST_CHECK_EQUAL(g2.getConfig().withDivision, g.getConfig().withDivision);
    BOOST_CHECK_EQUAL(g2.getSinkNode().getNumInArcs(), g.getSinkNode().getNumInArcs());
    BOOST_CHECK_EQUAL(g2.getSourceNode().getNumOutArcs(), g.getSourceNode().getNumOutArcs());
}


