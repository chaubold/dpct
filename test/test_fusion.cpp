#define BOOST_TEST_MODULE test_fusion

#include <iostream>
#include <boost/test/unit_test.hpp>
#include "graph.h"
#include "fusionmove.h"
#include "magnusson.h"

using namespace dpct;


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

BOOST_AUTO_TEST_CASE(fuse_solutions)
{
    // create graph
    Graph::Configuration config(true, true, true);
    Graph g(config);
    buildGraph(g);

    // create solution A
    Graph gA(g);
    Magnusson trackerA(&gA, false);
    TrackingAlgorithm::Solution pathsA;
    double scoreA = trackerA.track(pathsA);
    pathsA = trackerA.translateToOriginGraph(pathsA);
    std::cout << "Solution A with score " << scoreA << " has " << pathsA.size() << " paths" << std::endl;

    // create solution B (pick second best)
    Graph gB(g);
    Magnusson trackerB(&gB, false);
    trackerB.setPathStartSelectorFunction(selectSecondBestInArc);
    TrackingAlgorithm::Solution pathsB;
    double scoreB = trackerB.track(pathsB);
    pathsB = trackerB.translateToOriginGraph(pathsB);
    std::cout << "Solution B with score " << scoreB << " has " << pathsB.size() << " paths" << std::endl;

    // create graph union
    FusionMove fm(&g);
    std::shared_ptr<Graph> unionGraph = fm.graphUnion(pathsA, pathsB);
    BOOST_CHECK(unionGraph->getSinkNode().getNumInArcs() >= 1);

    // track on graph union
    Magnusson trackerFM(unionGraph.get(), false);
    TrackingAlgorithm::Solution pathsFM;
    double scoreFM = trackerFM.track(pathsFM);
    std::cout << "Solution FM with score " << scoreFM << " has " << pathsFM.size() << " paths" << std::endl;

    std::cout << "Original graph has " << g.getNumArcs() << " arcs and " << g.getNumNodes() << " nodes.\n";
    std::cout << "Union graph has " << unionGraph->getNumArcs()
              << " arcs and " << unionGraph->getNumNodes() << " nodes.\n" << std::endl;

    BOOST_CHECK(scoreFM >= scoreA);
    BOOST_CHECK(scoreFM >= scoreB);
    BOOST_CHECK(unionGraph->getNumArcs() <= g.getNumArcs());
    BOOST_CHECK(unionGraph->getNumNodes() <= g.getNumNodes());
    BOOST_CHECK(unionGraph->getNumTimesteps() == g.getNumTimesteps());
}

BOOST_AUTO_TEST_CASE(fuse_and_contract_solutions)
{
    std::vector<size_t> pathLengthsA = {5, 5, 5, 4, 3};
    std::vector<size_t> pathLengthsB = {5, 5, 4, 5, 5};

    auto checkPathLengths = [](const TrackingAlgorithm::Solution& s, const std::vector<size_t>& expectedLengths)
    {
        for(size_t i = 0; i < s.size(); i++)
        {
            BOOST_CHECK_EQUAL(s[0].size(), expectedLengths[0]);
        }
    };

    // create graph
    Graph::Configuration config(true, true, true);
    Graph g(config);
    buildGraph(g);

    // create solution A
    Graph gA(g);
    Magnusson trackerA(&gA, false);
    TrackingAlgorithm::Solution pathsA;
    double scoreA = trackerA.track(pathsA);
    pathsA = trackerA.translateToOriginGraph(pathsA);
    std::cout << "Solution A with score " << scoreA << " has " << pathsA.size() << " paths" << std::endl;
    BOOST_CHECK_EQUAL(pathsA.size(), 5);
    checkPathLengths(pathsA, pathLengthsA);

    // create solution B (pick second best)
    Graph gB(g);
    Magnusson trackerB(&gB, false);
    trackerB.setPathStartSelectorFunction(selectSecondBestInArc);
    TrackingAlgorithm::Solution pathsB;
    double scoreB = trackerB.track(pathsB);
    pathsB = trackerB.translateToOriginGraph(pathsB);
    std::cout << "Solution B with score " << scoreB << " has " << pathsB.size() << " paths" << std::endl;
    BOOST_CHECK_EQUAL(pathsB.size(), 5);
    checkPathLengths(pathsB, pathLengthsB);

    // create graph union
    FusionMove fm(&g);
    std::shared_ptr<Graph> unionGraph = fm.graphUnion(pathsA, pathsB);
    BOOST_CHECK(unionGraph->getSinkNode().getNumInArcs() >= 1);
    unionGraph->contractLoneArcs(false);

    // track on graph union
    Magnusson trackerFM(unionGraph.get(), false);
    TrackingAlgorithm::Solution pathsFM;
    double scoreFM = trackerFM.track(pathsFM);
    std::cout << "Solution FM with score " << scoreFM << " has " << pathsFM.size() << " paths" << std::endl;
    BOOST_CHECK_EQUAL(pathsFM.size(), 5);
    for(TrackingAlgorithm::Path& p : pathsFM)
    {
        BOOST_CHECK(p.size() <= 6);
    }

    pathsFM = trackerFM.translateToOriginGraph(pathsFM);
    checkPathLengths(pathsFM, pathLengthsA);


    std::cout << "Original graph has " << g.getNumArcs() << " arcs and " << g.getNumNodes() << " nodes.\n";
    std::cout << "Union graph has " << unionGraph->getNumArcs()
              << " arcs and " << unionGraph->getNumNodes() << " nodes.\n" << std::endl;

    BOOST_CHECK(scoreFM >= scoreA);
    BOOST_CHECK(scoreFM >= scoreB);
    BOOST_CHECK(unionGraph->getNumArcs() <= g.getNumArcs());
    BOOST_CHECK(unionGraph->getNumNodes() <= g.getNumNodes());
    BOOST_CHECK(unionGraph->getNumTimesteps() == g.getNumTimesteps());
}
