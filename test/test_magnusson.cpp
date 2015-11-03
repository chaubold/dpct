#define BOOST_TEST_MODULE test_magnusson

#include <iostream>
#include <boost/test/unit_test.hpp>
#include "graph.h"
#include "magnusson.h"

using namespace dpct;

BOOST_AUTO_TEST_CASE(minimal_test_magnusson)
{
    Graph::Configuration config(false, false, false);
    Graph g(config);

    Graph::NodePtr n1 = g.addNode(0, {0, 3,-10}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 1"));
    Graph::NodePtr n2 = g.addNode(1, {0, 3,-10}, 0.0, 0.0, false, true, std::make_shared<NameData>("Timestep 2: Node 1"));

    g.addMoveArc(n1, n2, 1.0);

    Magnusson tracker(&g, false);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    std::cout << "Tracker returned score " << score << std::endl;
    BOOST_CHECK_EQUAL(score, 7.0);
    BOOST_CHECK_EQUAL(paths.size(), 1);
}

BOOST_AUTO_TEST_CASE(two_cell_test_magnusson)
{
    Graph::Configuration config(false, false, false);
    Graph g(config);

    Graph::NodePtr n1 = g.addNode(0, {0, 3, 5, -10}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 1"));
    Graph::NodePtr n2 = g.addNode(1, {0, 3, 4, -10}, 0.0, 0.0, false, true, std::make_shared<NameData>("Timestep 2: Node 1"));

    g.addMoveArc(n1, n2, 1.0);

    Magnusson tracker(&g, false);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    std::cout << "Tracker returned score " << score << std::endl;
    BOOST_CHECK_EQUAL(score, 10.0); // arc score is applied only once!
    BOOST_CHECK_EQUAL(paths.size(), 2);
}

BOOST_AUTO_TEST_CASE(two_cell_test_arc_once_magnusson)
{
    Graph::Configuration config(false, false, false);
    Graph g(config);

    Graph::NodePtr n1 = g.addNode(0, {0, 3, 5, -10}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 1"));
    Graph::NodePtr n2 = g.addNode(1, {0, 3, 4, -10}, 0.0, 0.0, false, true, std::make_shared<NameData>("Timestep 2: Node 1"));

    g.addMoveArc(n1, n2, 1.0);

    Magnusson tracker(&g, false, true);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    std::cout << "Tracker returned score " << score << std::endl;
    BOOST_CHECK_EQUAL(score, 10.0);
    BOOST_CHECK_EQUAL(paths.size(), 2);
}

BOOST_AUTO_TEST_CASE(magnusson_no_swap_failure_case)
{
    Graph::Configuration config(false, false, false);
    Graph g(config);

    Graph::NodePtr n1 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 1"));
    Graph::NodePtr n2 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 2"));

    Graph::NodePtr n3 = g.addNode(1, {0, 5}, 0.0, 0.0, false, false, std::make_shared<NameData>("Timestep 2: Node 1"));
    Graph::NodePtr n4 = g.addNode(1, {0, 15}, 0.0, 0.0, false, false, std::make_shared<NameData>("Timestep 2: Node 2"));

    Graph::NodePtr n5 = g.addNode(2, {0, 2}, 0.0, 0.0, false, true, std::make_shared<NameData>("Timestep 3: Node 1"));
    Graph::NodePtr n6 = g.addNode(2, {0, 1}, 0.0, 0.0, false, true, std::make_shared<NameData>("Timestep 3: Node 2"));

    g.addMoveArc(n1, n3, 0.0);
    g.addMoveArc(n2, n4, 0.0);
    g.addMoveArc(n3, n5, 4.0);
    g.addMoveArc(n4, n5, 0.0);
    g.addMoveArc(n4, n6, 0.0);

    Magnusson tracker(&g, false);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    std::cout << "Tracker returned score " << score << std::endl;
    BOOST_CHECK_EQUAL(score, 18.0);
    BOOST_CHECK_EQUAL(paths.size(), 1);
}

BOOST_AUTO_TEST_CASE(magnusson_simple_swap_test)
{
    Graph::Configuration config(false, false, false);
    Graph g(config);

    Graph::NodePtr n1 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 1"));
    Graph::NodePtr n2 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 2"));

    Graph::NodePtr n3 = g.addNode(1, {0, 5}, 0.0, 0.0, false, false, std::make_shared<NameData>("Timestep 2: Node 1"));
    Graph::NodePtr n4 = g.addNode(1, {0, 15}, 0.0, 0.0, false, false, std::make_shared<NameData>("Timestep 2: Node 2"));

    Graph::NodePtr n5 = g.addNode(2, {0, 2}, 0.0, 0.0, false, true, std::make_shared<NameData>("Timestep 3: Node 1"));
    Graph::NodePtr n6 = g.addNode(2, {0, 1}, 0.0, 0.0, false, true, std::make_shared<NameData>("Timestep 3: Node 2"));

    g.addMoveArc(n1, n3, 0.0);
    g.addMoveArc(n2, n4, 0.0);
    g.addMoveArc(n3, n5, 4.0);
    g.addMoveArc(n4, n5, 0.0);
    g.addMoveArc(n4, n6, 0.0);

    Magnusson tracker(&g, true, true);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    std::cout << "Tracker returned score " << score << std::endl;
    BOOST_CHECK_EQUAL(score, 29.0);
    BOOST_CHECK_EQUAL(paths.size(), 2);

    // check the paths
    for(TrackingAlgorithm::Path& p : paths)
    {
        for(Arc* a : p)
        {
            BOOST_CHECK_NE(a->getType(), Arc::Swap);
            BOOST_CHECK(a->getSourceNode() != n4.get() || a->getTargetNode() != n5.get());
        }
    }
}

BOOST_AUTO_TEST_CASE(magnusson_simple_swap_test_with_app_dis)
{
    Graph::Configuration config(true, true, false);
    Graph g(config);

    Graph::NodePtr n1 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 1"));
    Graph::NodePtr n2 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 2"));

    Graph::NodePtr n3 = g.addNode(1, {0, 10}, 10.0, 0.0, false, true, std::make_shared<NameData>("Timestep 2: Node 1"));
    Graph::NodePtr n4 = g.addNode(1, {0, 1}, 0.0, 0.0, false, true, std::make_shared<NameData>("Timestep 2: Node 2"));

    g.addMoveArc(n1, n3, 0.0);
    g.addMoveArc(n1, n4, 2.0);
    g.addMoveArc(n2, n4, 2.0);

    Magnusson tracker(&g, true, true);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    std::cout << "Tracker returned score " << score << std::endl;
    // BOOST_CHECK_EQUAL(score, 29.0);
    // BOOST_CHECK_EQUAL(paths.size(), 2);
}

BOOST_AUTO_TEST_CASE(magnusson_two_possible_swaps)
{
    Graph::Configuration config(false, false, false);
    Graph g(config);

    Graph::NodePtr n1 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 1"));
    Graph::NodePtr n2 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 2"));
    Graph::NodePtr n10 = g.addNode(0, {0, 2}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 3"));

    Graph::NodePtr n3 = g.addNode(1, {0, 5}, 0.0, 0.0, false, false, std::make_shared<NameData>("Timestep 2: Node 1"));
    Graph::NodePtr n4 = g.addNode(1, {0, 15}, 0.0, 0.0, false, false, std::make_shared<NameData>("Timestep 2: Node 2"));
    Graph::NodePtr n11 = g.addNode(0, {0, 2}, 0.0, 0.0, false, false, std::make_shared<NameData>("Timestep 2: Node 3"));

    Graph::NodePtr n5 = g.addNode(2, {0, 2}, 0.0, 0.0, false, true, std::make_shared<NameData>("Timestep 3: Node 1"));
    Graph::NodePtr n6 = g.addNode(2, {0, 1}, 0.0, 0.0, false, true, std::make_shared<NameData>("Timestep 3: Node 2"));
    Graph::NodePtr n12 = g.addNode(2, {0, 1}, 0.0, 0.0, false, true, std::make_shared<NameData>("Timestep 3: Node 3"));

    g.addMoveArc(n1, n3, 0.0);
    g.addMoveArc(n2, n4, 0.0);
    g.addMoveArc(n3, n5, 4.0);
    g.addMoveArc(n4, n5, 0.0);
    g.addMoveArc(n4, n6, 0.0);
    g.addMoveArc(n10, n11, 0.0);
    g.addMoveArc(n11, n12, 0.0);
    g.addMoveArc(n11, n5, 0.0);
    g.addMoveArc(n4, n12, 0.0);

    Magnusson tracker(&g, true, true);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    std::cout << "Tracker returned score " << score << std::endl;
    BOOST_CHECK_EQUAL(score, 34.0);
    BOOST_CHECK_EQUAL(paths.size(), 3);
}

void buildGraph(Graph& g)
{
    double appearanceScore = -200;
    double disappearanceScore = -200;
    double divisionScore = -200;

    // -----------------------------------------------------
    // Timestep 1
    Graph::NodePtr n_1_1 = g.addNode(0, {0, 3, 2}, appearanceScore, disappearanceScore,
                                        true, false, std::make_shared<NameData>("Timestep 1: Node 1"));

    Graph::NodePtr n_1_2 = g.addNode(0, {0, 2, 7}, appearanceScore, disappearanceScore,
                                        true, false, std::make_shared<NameData>("Timestep 1: Node 2"));

    Graph::NodePtr n_1_3 = g.addNode(0, {0, 3, -2}, appearanceScore, disappearanceScore,
                                        true, false, std::make_shared<NameData>("Timestep 1: Node 3"));

    // -----------------------------------------------------
    // Timestep 2
    Graph::NodePtr n_2_1 = g.addNode(1, {0, 4, 2}, appearanceScore, disappearanceScore,
                                        false, false, std::make_shared<NameData>("Timestep 2: Node 1"));

    Graph::NodePtr n_2_2 = g.addNode(1, {0, 2, 0}, appearanceScore, disappearanceScore,
                                        false, false, std::make_shared<NameData>("Timestep 2: Node 2"));

    Graph::NodePtr n_2_3 = g.addNode(1, {0, 2, -2}, appearanceScore, disappearanceScore,
                                        false, false, std::make_shared<NameData>("Timestep 2: Node 3"));

    Graph::NodePtr n_2_4 = g.addNode(1, {0, 2, 0}, appearanceScore, -1.0,
                                        false, false, std::make_shared<NameData>("Timestep 2: Node 4"));

    g.addMoveArc(n_1_1, n_2_1, 0.0);
    g.addMoveArc(n_1_2, n_2_2, 0.0);
    g.addMoveArc(n_1_2, n_2_3, 0.0);
    g.addMoveArc(n_1_3, n_2_4, 0.0);

    // -----------------------------------------------------
    // Timestep 3
    Graph::NodePtr n_3_1 = g.addNode(2, {0, 3, 5}, appearanceScore, disappearanceScore,
                                        false, false, std::make_shared<NameData>("Timestep 3: Node 1"));

    Graph::NodePtr n_3_2 = g.addNode(2, {0, 2, 2}, appearanceScore, disappearanceScore,
                                        false, false, std::make_shared<NameData>("Timestep 3: Node 2"));

    Graph::NodePtr n_3_3 = g.addNode(2, {0, 3, 0}, appearanceScore, disappearanceScore,
                                        false, false, std::make_shared<NameData>("Timestep 3: Node 3"));


    g.addMoveArc(n_2_1, n_3_1, 0.0);
    g.addMoveArc(n_2_2, n_3_2, 0.0);
    g.addMoveArc(n_2_3, n_3_3, 0.0);

    // -----------------------------------------------------
    // Timestep 4
    Graph::NodePtr n_4_1 = g.addNode(3, {0, 4, 3}, appearanceScore, disappearanceScore,
                                        false, true, std::make_shared<NameData>("Timestep 4: Node 1"));

    Graph::NodePtr n_4_2 = g.addNode(3, {0, 2, 1}, appearanceScore, disappearanceScore,
                                        false, true, std::make_shared<NameData>("Timestep 4: Node 2"));

    Graph::NodePtr n_4_3 = g.addNode(3, {0, 2, -4}, appearanceScore, disappearanceScore,
                                        false, true, std::make_shared<NameData>("Timestep 4: Node 3"));

    Graph::NodePtr n_4_4 = g.addNode(3, {0, 4, 2}, appearanceScore, disappearanceScore,
                                        false, true, std::make_shared<NameData>("Timestep 4: Node 4"));

    g.addMoveArc(n_3_1, n_4_1, 0.0);
    g.addMoveArc(n_3_1, n_4_2, 0.0);
    g.addMoveArc(n_3_2, n_4_2, 0.0);
    g.addMoveArc(n_3_2, n_4_3, 0.0);
    g.addMoveArc(n_3_3, n_4_3, 0.0);
    g.addMoveArc(n_3_3, n_4_4, 0.0);

    g.allowMitosis(n_3_2, n_4_3, -1);
    g.allowMitosis(n_3_3, n_4_3, 2);

    std::cout << "Done setting up graph" << std::endl;
}

BOOST_AUTO_TEST_CASE(test_full_magnusson_no_swap)
{
    Graph::Configuration config(true, true, true);
    Graph g(config);

    buildGraph(g);
	
    // -----------------------------------------------------
    // Tracking
    Magnusson tracker(&g, false, true);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    BOOST_CHECK_EQUAL(paths.size(), 5);
    BOOST_CHECK_EQUAL(score, 44.0);

    std::cout << "Tracker returned score " << score << std::endl;
}


BOOST_AUTO_TEST_CASE(test_full_magnusson_with_swap)
{
    Graph::Configuration config(true, true, true);
    Graph g(config);

    buildGraph(g);

    // -----------------------------------------------------
    // Tracking
    Magnusson tracker(&g, true, true);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    BOOST_CHECK_EQUAL(paths.size(), 5);
    BOOST_CHECK_EQUAL(score, 44.0);

    std::cout << "Tracker returned score " << score << std::endl;
}

BOOST_AUTO_TEST_CASE(test_full_magnusson_graph_constness)
{
    Graph::Configuration config(true, true, true);
    Graph g(config);

    buildGraph(g);

    size_t num_arcs = g.getNumArcs();
    size_t num_timesteps = g.getNumTimesteps();
    size_t num_nodes = g.getNumNodes();

    // -----------------------------------------------------
    // Tracking
    Magnusson tracker(&g, true, true);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    BOOST_CHECK_EQUAL(g.getNumArcs(), num_arcs);
    BOOST_CHECK_EQUAL(g.getNumNodes(), num_nodes);
    BOOST_CHECK_EQUAL(g.getNumTimesteps(), num_timesteps);
}

void updateNode(Node* n)
{
    n->updateBestInArcAndScore();
    
    for(Node::ArcIt outArc = n->getOutArcsBegin(); outArc != n->getOutArcsEnd(); ++outArc)
    {
        (*outArc)->update();
    }
}

BOOST_AUTO_TEST_CASE(test_backward_path)
{
    using std::placeholders::_1;
    Graph::Configuration config(true, true, true);
    Graph g(config);

    buildGraph(g);

    size_t num_arcs = g.getNumArcs();
    size_t num_timesteps = g.getNumTimesteps();
    size_t num_nodes = g.getNumNodes();

    // -----------------------------------------------------
    // Tracking
    Magnusson tracker(&g, true, true);
    // init best in arcs
    for(size_t t = 0; t < g.getNumTimesteps(); ++t)
    {
        g.visitNodesInTimestep(t, std::bind(&updateNode, _1));
    }
    g.visitSpecialNodes(std::bind(&updateNode, _1));

    TrackingAlgorithm::Solution paths;
    tracker.findNonintersectingBackwardPaths(&g.getSourceNode(), &g.getSinkNode(), paths);

    BOOST_CHECK(paths.size() > 0);
    BOOST_CHECK(paths.size() <= g.getSinkNode().getNumInArcs());

    // test that none of the paths overlap
    std::map<const Arc*, size_t> arcUseCount;
    for(TrackingAlgorithm::Path& p : paths)
    {
        for(const Arc* a : p)
        {
            BOOST_CHECK(arcUseCount.find(a) == arcUseCount.end());
            arcUseCount[a] = 1;
        }
    }
}

BOOST_AUTO_TEST_CASE(test_full_magnusson_fast_1st_iter)
{
    Graph::Configuration config(true, true, true);
    Graph g(config);

    buildGraph(g);
    
    // -----------------------------------------------------
    // Tracking
    Magnusson tracker(&g, false, true, true);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    BOOST_CHECK(paths.size() <= 5);
    BOOST_CHECK(score <= 44.0);
    BOOST_CHECK(score >= 0);

    std::cout << "Tracker returned score " << score << std::endl;

    // Magnusson tracker2(&g, false, false, false);
    // std::vector<TrackingAlgorithm::Path> paths2;
    // score = tracker2.track(paths2);
    // BOOST_CHECK(tracker.getElapsedSeconds() < tracker2.getElapsedSeconds());
}

BOOST_AUTO_TEST_CASE(test_magnusson_maxNumPaths)
{
    Graph::Configuration config(true, true, true);
    Graph g(config);

    buildGraph(g);
    
    // -----------------------------------------------------
    // Tracking
    Magnusson tracker(&g, false, true, true);
    std::vector<TrackingAlgorithm::Path> paths;
    tracker.setMaxNumberOfPaths(2);
    double score = tracker.track(paths);

    BOOST_CHECK(paths.size() == 2);
    BOOST_CHECK(score <= 44.0);
    BOOST_CHECK(score >= 0);

    std::cout << "Tracker returned score " << score << std::endl;

    // Magnusson tracker2(&g, false, false, false);
    // std::vector<TrackingAlgorithm::Path> paths2;
    // score = tracker2.track(paths2);
    // BOOST_CHECK(tracker.getElapsedSeconds() < tracker2.getElapsedSeconds());
}

BOOST_AUTO_TEST_CASE(magnusson_selector_func)
{
    Graph::Configuration config(false, false, false);
    Graph g(config);

    Graph::NodePtr n1 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 1"));
    Graph::NodePtr n2 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 2"));

    Graph::NodePtr n3 = g.addNode(1, {0, 5}, 0.0, 0.0, false, false, std::make_shared<NameData>("Timestep 2: Node 1"));
    Graph::NodePtr n4 = g.addNode(1, {0, 15}, 0.0, 0.0, false, false, std::make_shared<NameData>("Timestep 2: Node 2"));

    Graph::NodePtr n5 = g.addNode(2, {0, 2}, 0.0, 0.0, false, true, std::make_shared<NameData>("Timestep 3: Node 1"));
    Graph::NodePtr n6 = g.addNode(2, {0, 1}, 0.0, 0.0, false, true, std::make_shared<NameData>("Timestep 3: Node 2"));

    g.addMoveArc(n1, n3, 0.0);
    g.addMoveArc(n2, n4, 0.0);
    g.addMoveArc(n3, n5, 4.0);
    g.addMoveArc(n4, n5, 0.0);
    g.addMoveArc(n4, n6, 0.0);

    Magnusson tracker(&g, false);
    tracker.setPathStartSelectorFunction([](Node* n) -> Arc* {
        Node::ArcIt it = n->getInArcsBegin() + (n->getNumInArcs() - 1);
        return *it;
    });
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    std::cout << "Tracker returned score " << score << std::endl;
    BOOST_CHECK_EQUAL(score, 17.0);
    BOOST_CHECK_EQUAL(paths.size(), 1);
}

BOOST_AUTO_TEST_CASE(magnusson_selector_func_2nd_best)
{
    Graph::Configuration config(false, false, false);
    Graph g(config);

    Graph::NodePtr n1 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 1"));
    Graph::NodePtr n2 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 2"));

    Graph::NodePtr n3 = g.addNode(1, {0, 5}, 0.0, 0.0, false, false, std::make_shared<NameData>("Timestep 2: Node 1"));
    Graph::NodePtr n4 = g.addNode(1, {0, 15}, 0.0, 0.0, false, false, std::make_shared<NameData>("Timestep 2: Node 2"));

    Graph::NodePtr n5 = g.addNode(2, {0, 2}, 0.0, 0.0, false, true, std::make_shared<NameData>("Timestep 3: Node 1"));
    Graph::NodePtr n6 = g.addNode(2, {0, 1}, 0.0, 0.0, false, true, std::make_shared<NameData>("Timestep 3: Node 2"));

    g.addMoveArc(n1, n3, 0.0);
    g.addMoveArc(n2, n4, 0.0);
    g.addMoveArc(n3, n5, 4.0);
    g.addMoveArc(n4, n5, 0.0);
    g.addMoveArc(n4, n6, 0.0);

    Magnusson tracker(&g, false);
    tracker.setPathStartSelectorFunction(selectSecondBestInArc);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    std::cout << "Tracker returned score " << score << std::endl;
    BOOST_CHECK_EQUAL(score, 29.0);
    BOOST_CHECK_EQUAL(paths.size(), 2);
}

BOOST_AUTO_TEST_CASE(magnusson_selector_func_random)
{
    Graph::Configuration config(false, false, false);
    Graph g(config);

    Graph::NodePtr n1 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 1"));
    Graph::NodePtr n2 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, std::make_shared<NameData>("Timestep 1: Node 2"));

    Graph::NodePtr n3 = g.addNode(1, {0, 5}, 0.0, 0.0, false, false, std::make_shared<NameData>("Timestep 2: Node 1"));
    Graph::NodePtr n4 = g.addNode(1, {0, 15}, 0.0, 0.0, false, false, std::make_shared<NameData>("Timestep 2: Node 2"));

    Graph::NodePtr n5 = g.addNode(2, {0, 2}, 0.0, 0.0, false, true, std::make_shared<NameData>("Timestep 3: Node 1"));
    Graph::NodePtr n6 = g.addNode(2, {0, 1}, 0.0, 0.0, false, true, std::make_shared<NameData>("Timestep 3: Node 2"));

    g.addMoveArc(n1, n3, 0.0);
    g.addMoveArc(n2, n4, 0.0);
    g.addMoveArc(n3, n5, 4.0);
    g.addMoveArc(n4, n5, 0.0);
    g.addMoveArc(n4, n6, 0.0);

    Magnusson tracker(&g, false);
    tracker.setPathStartSelectorFunction(selectAtRandom);
    std::vector<TrackingAlgorithm::Path> paths;
    double score = tracker.track(paths);

    std::cout << "Tracker returned score " << score << std::endl;
    BOOST_CHECK(score >= 17.0);
    BOOST_CHECK(paths.size() >= 1);
}


class PositionData2D : public UserData
{
public:
    PositionData2D(double x, double y):
        x_(x),
        y_(y)
    {}

    virtual std::string toString() const 
    { 
        std::stringstream s;
        s << "Pos(" << x_ << ", " << y_ << ")"; 
        return s.str();
    }

    double X() const { return x_; }
    double Y() const { return y_; }
private:
    double x_, y_;
};

/*
// TODO: fix swap arc handling when motion model is present!
BOOST_AUTO_TEST_CASE(test_full_magnusson_motion_model)
{
    Graph::Configuration config(true, true, true);
    Graph g(config);

    Graph::NodePtr n1 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, std::make_shared<PositionData2D>(0,0));
    Graph::NodePtr n2 = g.addNode(0, {0, 1}, 0.0, 0.0, true, false, std::make_shared<PositionData2D>(1,0));

    Graph::NodePtr n3 = g.addNode(1, {0, 5}, 0.0, 0.0, false, false, std::make_shared<PositionData2D>(0,1));
    Graph::NodePtr n4 = g.addNode(1, {0, 15}, 0.0, 0.0, false, false, std::make_shared<PositionData2D>(1,1));

    Graph::NodePtr n5 = g.addNode(2, {0, 2}, 0.0, 0.0, false, true, std::make_shared<PositionData2D>(0,2));
    Graph::NodePtr n6 = g.addNode(2, {0, 1}, 0.0, 0.0, false, true, std::make_shared<PositionData2D>(1,2));

    g.addMoveArc(n1, n3, 0.0);
    g.addMoveArc(n2, n4, 0.0);
    g.addMoveArc(n3, n5, 4.0);
    g.addMoveArc(n4, n5, 0.0);
    g.addMoveArc(n4, n6, 0.0);

    // set up motion model
    Magnusson::MotionModelScoreFunction momoscofu = [&](Node* a, Node* b, Node* c)
    {
        typedef std::shared_ptr<PositionData2D> Pos2DPtr;

        // only evaluate if we're looking at 3 proper nodes, otherwise return default value
        if(a == nullptr || b == nullptr || c == nullptr || g.isSpecialNode(a) || g.isSpecialNode(b) || g.isSpecialNode(c))
        {
            return -10.0;
        }

        Pos2DPtr posA = std::static_pointer_cast<PositionData2D>(a->getUserData());
        Pos2DPtr posB = std::static_pointer_cast<PositionData2D>(b->getUserData());
        Pos2DPtr posC = std::static_pointer_cast<PositionData2D>(c->getUserData());
        double dist1 = sqrt( pow(posA->X() - posB->X(), 2.0) + pow(posA->Y() - posB->Y(), 2.0) );
        double dist2 = sqrt( pow(posC->X() - posB->X(), 2.0) + pow(posC->Y() - posB->Y(), 2.0) );

        double velocityMagnitude = fabsf(dist1 - dist2);
        std::cout << "Points: {" << posA->toString() << ", " << posB->toString() << ", " << posC->toString()
                  << "} have distances " << dist1 << ", " << dist2 << " and vel " << velocityMagnitude << std::endl;
        return -velocityMagnitude;
    };
    
    // -----------------------------------------------------
    // Tracking
    Magnusson tracker(&g, false, false);
    std::vector<TrackingAlgorithm::Path> paths;
    tracker.setMotionModelScoreFunction(momoscofu);
    double score = tracker.track(paths);

    BOOST_CHECK_EQUAL(paths.size(), 3);
    BOOST_CHECK(score < 24.6 && score > 24.5); // should be 25 - (sqrt(2)-1)

    std::cout << "Tracker returned score " << score << std::endl;
}
*/
