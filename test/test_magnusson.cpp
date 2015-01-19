#define BOOST_TEST_MODULE test_graph

#include <iostream>
#include <boost/test/unit_test.hpp>
#include "graph.h"
#include "magnusson.h"

using namespace dpct;

BOOST_AUTO_TEST_CASE(minimal_test_magnusson)
{
    Graph::Configuration config(false, false, false, false);
    Graph g(config);

    NameData nd_1("Timestep 1: Node 1");
    Graph::NodePtr n1 = g.addNode(0, {3,-10}, 0.0, 0.0, true, false, &nd_1);
    NameData nd_2("Timestep 2: Node 1");
    Graph::NodePtr n2 = g.addNode(1, {3,-10}, 0.0, 0.0, false, true, &nd_2);

    g.addMoveArc(n1, n2, 1.0);

    Magnusson tracker(&g);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    std::cout << "Tracker returned score " << score << std::endl;
    BOOST_CHECK_EQUAL(score, 7.0);
    BOOST_CHECK_EQUAL(paths.size(), 1);
}

BOOST_AUTO_TEST_CASE(test_full_magnusson)
{
    Graph::Configuration config(true, true, true, false);
    Graph g(config);

    double appearanceScore = -200;
    double disappearanceScore = -200;
    double divisionScore = -200;

    // -----------------------------------------------------
    // Timestep 1
    NameData nd_1_1("Timestep 1: Node 1");
    Graph::NodePtr n_1_1 = g.addNode(0, {3, -1}, appearanceScore, disappearanceScore,
                                        true, false, &nd_1_1);

    NameData nd_1_2("Timestep 1: Node 2");
    Graph::NodePtr n_1_2 = g.addNode(0, {2, 5, -2}, appearanceScore, disappearanceScore,
                                        true, false, &nd_1_2);

    NameData nd_1_3("Timestep 1: Node 3");
    Graph::NodePtr n_1_3 = g.addNode(0, {3, -5}, appearanceScore, disappearanceScore,
                                        true, false, &nd_1_3);

    // -----------------------------------------------------
    // Timestep 2
    NameData nd_2_1("Timestep 2: Node 1");
    Graph::NodePtr n_2_1 = g.addNode(1, {4, -2}, appearanceScore, disappearanceScore,
                                        false, false, &nd_2_1);
	
    NameData nd_2_2("Timestep 2: Node 2");
    Graph::NodePtr n_2_2 = g.addNode(1, {2, -2}, appearanceScore, disappearanceScore,
                                        false, false, &nd_2_2);

    NameData nd_2_3("Timestep 2: Node 3");
    Graph::NodePtr n_2_3 = g.addNode(1, {2, -4}, appearanceScore, disappearanceScore,
                                        false, false, &nd_2_3);

    NameData nd_2_4("Timestep 2: Node 4");
    Graph::NodePtr n_2_4 = g.addNode(1, {2,-2}, appearanceScore, -1.0,
                                        false, false, &nd_2_4);

    NameData ad_2_1("Arc 1");
    g.addMoveArc(n_1_1, n_2_1, 0.0, &ad_2_1);
    g.addMoveArc(n_1_2, n_2_2, 0.0);
    g.addMoveArc(n_1_2, n_2_3, 0.0);
    g.addMoveArc(n_1_3, n_2_4, 0.0);

    // -----------------------------------------------------
    // Timestep 3
    NameData nd_3_1("Timestep 3: Node 1");
    Graph::NodePtr n_3_1 = g.addNode(2, {3, 2}, appearanceScore, disappearanceScore,
                                        false, false, &nd_3_1);

    NameData nd_3_2("Timestep 3: Node 2");
    Graph::NodePtr n_3_2 = g.addNode(2, {2, 0}, appearanceScore, disappearanceScore,
                                        false, false, &nd_3_2);

    NameData nd_3_3("Timestep 3: Node 3");
    Graph::NodePtr n_3_3 = g.addNode(2, {3, -3}, appearanceScore, disappearanceScore,
                                        false, false, &nd_3_3);


    g.addMoveArc(n_2_1, n_3_1, 0.0);
    g.addMoveArc(n_2_2, n_3_2, 0.0);
    g.addMoveArc(n_2_3, n_3_3, 0.0);

    // -----------------------------------------------------
    // Timestep 4
    NameData nd_4_1("Timestep 4: Node 1");
    Graph::NodePtr n_4_1 = g.addNode(3, {4, -1}, appearanceScore, disappearanceScore,
                                        false, true, &nd_4_1);

    NameData nd_4_2("Timestep 4: Node 2");
    Graph::NodePtr n_4_2 = g.addNode(3, {2, -1}, appearanceScore, disappearanceScore,
                                        false, true, &nd_4_2);

    NameData nd_4_3("Timestep 4: Node 3");
    Graph::NodePtr n_4_3 = g.addNode(3, {2, -6}, appearanceScore, disappearanceScore,
                                        false, true, &nd_4_3);

    NameData nd_4_4("Timestep 4: Node 4");
    Graph::NodePtr n_4_4 = g.addNode(3, {4, -2}, appearanceScore, disappearanceScore,
                                        false, true, &nd_4_4);

    g.addMoveArc(n_3_1, n_4_1, 0.0);
    g.addMoveArc(n_3_1, n_4_2, 0.0);
    g.addMoveArc(n_3_2, n_4_2, 0.0);
    g.addMoveArc(n_3_2, n_4_3, 0.0);
    g.addMoveArc(n_3_3, n_4_3, 0.0);
    g.addMoveArc(n_3_3, n_4_4, 0.0);

    g.allowMitosis(n_3_2, n_4_3, -1);
    g.allowMitosis(n_3_3, n_4_3, 2);

    std::cout << "Done setting up graph" << std::endl;
	
    // -----------------------------------------------------
    // Tracking
    Magnusson tracker(&g);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    BOOST_CHECK_EQUAL(paths.size(), 5);
    BOOST_CHECK_EQUAL(score, 44.0);

    std::cout << "Tracker returned score " << score << std::endl;
}
