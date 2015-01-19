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
    Graph::NodePtr n1 = g.addNode(0, {0, 3,-10}, 0.0, 0.0, true, false, &nd_1);
    NameData nd_2("Timestep 2: Node 1");
    Graph::NodePtr n2 = g.addNode(1, {0, 3,-10}, 0.0, 0.0, false, true, &nd_2);

    g.addMoveArc(n1, n2, 1.0);

    Magnusson tracker(&g);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    std::cout << "Tracker returned score " << score << std::endl;
    BOOST_CHECK_EQUAL(score, 7.0);
    BOOST_CHECK_EQUAL(paths.size(), 1);
}

BOOST_AUTO_TEST_CASE(two_cell_test_magnusson)
{
    Graph::Configuration config(false, false, false, false);
    Graph g(config);

    NameData nd_1("Timestep 1: Node 1");
    Graph::NodePtr n1 = g.addNode(0, {0, 3, 5, -10}, 0.0, 0.0, true, false, &nd_1);
    NameData nd_2("Timestep 2: Node 1");
    Graph::NodePtr n2 = g.addNode(1, {0, 3, 4, -10}, 0.0, 0.0, false, true, &nd_2);

    g.addMoveArc(n1, n2, 1.0);

    Magnusson tracker(&g);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    std::cout << "Tracker returned score " << score << std::endl;
    BOOST_CHECK_EQUAL(score, 11.0);
    BOOST_CHECK_EQUAL(paths.size(), 2);
}

BOOST_AUTO_TEST_CASE(magnusson_no_swap_failure_case)
{
    Graph::Configuration config(false, false, false, false);
    Graph g(config);

    NameData nd_1("Timestep 1: Node 1");
    Graph::NodePtr n1 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, &nd_1);
    NameData nd_2("Timestep 1: Node 2");
    Graph::NodePtr n2 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, &nd_2);

    NameData nd_3("Timestep 2: Node 1");
    Graph::NodePtr n3 = g.addNode(1, {0, 5}, 0.0, 0.0, false, false, &nd_3);
    NameData nd_4("Timestep 2: Node 2");
    Graph::NodePtr n4 = g.addNode(1, {0, 15}, 0.0, 0.0, false, false, &nd_4);

    NameData nd_5("Timestep 3: Node 1");
    Graph::NodePtr n5 = g.addNode(2, {0, 2}, 0.0, 0.0, false, true, &nd_5);
    NameData nd_6("Timestep 3: Node 2");
    Graph::NodePtr n6 = g.addNode(2, {0, 1}, 0.0, 0.0, false, true, &nd_6);

    g.addMoveArc(n1, n3, 0.0);
    g.addMoveArc(n2, n4, 0.0);
    g.addMoveArc(n3, n5, 4.0);
    g.addMoveArc(n4, n5, 0.0);
    g.addMoveArc(n4, n6, 0.0);

    Magnusson tracker(&g);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    std::cout << "Tracker returned score " << score << std::endl;
    BOOST_CHECK_EQUAL(score, 18.0);
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
    Graph::NodePtr n_1_1 = g.addNode(0, {0, 3, 2}, appearanceScore, disappearanceScore,
                                        true, false, &nd_1_1);

    NameData nd_1_2("Timestep 1: Node 2");
    Graph::NodePtr n_1_2 = g.addNode(0, {0, 2, 7}, appearanceScore, disappearanceScore,
                                        true, false, &nd_1_2);

    NameData nd_1_3("Timestep 1: Node 3");
    Graph::NodePtr n_1_3 = g.addNode(0, {0, 3, -2}, appearanceScore, disappearanceScore,
                                        true, false, &nd_1_3);

    // -----------------------------------------------------
    // Timestep 2
    NameData nd_2_1("Timestep 2: Node 1");
    Graph::NodePtr n_2_1 = g.addNode(1, {0, 4, 2}, appearanceScore, disappearanceScore,
                                        false, false, &nd_2_1);
	
    NameData nd_2_2("Timestep 2: Node 2");
    Graph::NodePtr n_2_2 = g.addNode(1, {0, 2, 0}, appearanceScore, disappearanceScore,
                                        false, false, &nd_2_2);

    NameData nd_2_3("Timestep 2: Node 3");
    Graph::NodePtr n_2_3 = g.addNode(1, {0, 2, -2}, appearanceScore, disappearanceScore,
                                        false, false, &nd_2_3);

    NameData nd_2_4("Timestep 2: Node 4");
    Graph::NodePtr n_2_4 = g.addNode(1, {0, 2, 0}, appearanceScore, -1.0,
                                        false, false, &nd_2_4);

    NameData ad_2_1("Arc 1");
    g.addMoveArc(n_1_1, n_2_1, 0.0, &ad_2_1);
    g.addMoveArc(n_1_2, n_2_2, 0.0);
    g.addMoveArc(n_1_2, n_2_3, 0.0);
    g.addMoveArc(n_1_3, n_2_4, 0.0);

    // -----------------------------------------------------
    // Timestep 3
    NameData nd_3_1("Timestep 3: Node 1");
    Graph::NodePtr n_3_1 = g.addNode(2, {0, 3, 5}, appearanceScore, disappearanceScore,
                                        false, false, &nd_3_1);

    NameData nd_3_2("Timestep 3: Node 2");
    Graph::NodePtr n_3_2 = g.addNode(2, {0, 2, 2}, appearanceScore, disappearanceScore,
                                        false, false, &nd_3_2);

    NameData nd_3_3("Timestep 3: Node 3");
    Graph::NodePtr n_3_3 = g.addNode(2, {0, 3, 0}, appearanceScore, disappearanceScore,
                                        false, false, &nd_3_3);


    g.addMoveArc(n_2_1, n_3_1, 0.0);
    g.addMoveArc(n_2_2, n_3_2, 0.0);
    g.addMoveArc(n_2_3, n_3_3, 0.0);

    // -----------------------------------------------------
    // Timestep 4
    NameData nd_4_1("Timestep 4: Node 1");
    Graph::NodePtr n_4_1 = g.addNode(3, {0, 4, 3}, appearanceScore, disappearanceScore,
                                        false, true, &nd_4_1);

    NameData nd_4_2("Timestep 4: Node 2");
    Graph::NodePtr n_4_2 = g.addNode(3, {0, 2, 1}, appearanceScore, disappearanceScore,
                                        false, true, &nd_4_2);

    NameData nd_4_3("Timestep 4: Node 3");
    Graph::NodePtr n_4_3 = g.addNode(3, {0, 2, -4}, appearanceScore, disappearanceScore,
                                        false, true, &nd_4_3);

    NameData nd_4_4("Timestep 4: Node 4");
    Graph::NodePtr n_4_4 = g.addNode(3, {0, 4, 2}, appearanceScore, disappearanceScore,
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
