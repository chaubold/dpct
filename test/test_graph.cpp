#define BOOST_TEST_MODULE test_graph

#include <iostream>
#include <boost/test/unit_test.hpp>
#include "graph.h"

using namespace dpct;

BOOST_AUTO_TEST_CASE(build_graph)
{
	Graph::Configuration config(true, true, true, false);
	Graph g(config);
	BOOST_CHECK_EQUAL(g.getNumArcs(), 3);
	BOOST_CHECK_EQUAL(g.getNumNodes(), 0);

	double appearanceScore = -200;
	double disappearanceScore = -200;
	double divisionScore = -200;

	// -----------------------------------------------------
	// Timestep 1
	NameData nd_1_1("Timestep 1: Node 1");
	Graph::NodePtr n_1_1 = g.addNode({3, -1}, appearanceScore, disappearanceScore, 
										true, false, &nd_1_1);
	// std::cout << "Inserted node: " << *(NameData*)(n_1_1->getUserData()) << std::endl;

	NameData nd_1_2("Timestep 1: Node 2");
	Graph::NodePtr n_1_2 = g.addNode({2, 5, -2}, appearanceScore, disappearanceScore, 
										true, false, &nd_1_2);

	NameData nd_1_3("Timestep 1: Node 3");
	Graph::NodePtr n_1_3 = g.addNode({3, -5}, appearanceScore, disappearanceScore, 
										true, false, &nd_1_3);

	BOOST_CHECK_EQUAL(g.getNumArcs(), 6);
	BOOST_CHECK_EQUAL(g.getNumNodes(), 3);

	// -----------------------------------------------------
	// Timestep 2
	NameData nd_2_1("Timestep 2: Node 1");
	Graph::NodePtr n_2_1 = g.addNode({4, -2}, appearanceScore, disappearanceScore, 
										false, false, &nd_2_1);
	
	NameData nd_2_2("Timestep 2: Node 2");
	Graph::NodePtr n_2_2 = g.addNode({2, -2}, appearanceScore, disappearanceScore, 
										false, false, &nd_2_2);

	NameData nd_2_3("Timestep 2: Node 3");
	Graph::NodePtr n_2_3 = g.addNode({2, -4}, appearanceScore, disappearanceScore, 
										false, false, &nd_2_3);

	NameData nd_2_4("Timestep 2: Node 4");
	Graph::NodePtr n_2_4 = g.addNode({2,-2}, appearanceScore, disappearanceScore, 
										false, false, &nd_2_4);

	BOOST_CHECK_EQUAL(g.getNumArcs(), 14);
	BOOST_CHECK_EQUAL(g.getNumNodes(), 7);

	NameData ad_2_1("Arc 1");
	g.addMoveArc(n_1_1, n_2_1, 0.0, &ad_2_1);
	g.addMoveArc(n_1_2, n_2_2, 0.0);
	g.addMoveArc(n_1_2, n_2_3, 0.0);
	g.addMoveArc(n_1_3, n_2_4, 0.0);

	BOOST_CHECK_EQUAL(g.getNumArcs(), 18);
	BOOST_CHECK_EQUAL(g.getNumNodes(), 7);

	// -----------------------------------------------------
	// Timestep 3
	Graph::NodePtr n_3_1 = g.addNode({3, 2}, appearanceScore, disappearanceScore, 
										false, false);

	Graph::NodePtr n_3_2 = g.addNode({2, 0}, appearanceScore, disappearanceScore, 
										false, false);

	Graph::NodePtr n_3_3 = g.addNode({3, -3}, appearanceScore, disappearanceScore, 
										false, false);

	BOOST_CHECK_EQUAL(g.getNumArcs(), 24);
	BOOST_CHECK_EQUAL(g.getNumNodes(), 10);

	g.addMoveArc(n_2_1, n_3_1, 0.0);
	g.addMoveArc(n_2_2, n_3_2, 0.0);
	g.addMoveArc(n_2_3, n_3_3, 0.0);
	BOOST_CHECK_EQUAL(g.getNumArcs(), 27);

	// -----------------------------------------------------
	// Timestep 4
	Graph::NodePtr n_4_1 = g.addNode({4, -1}, appearanceScore, disappearanceScore, 
										false, true);

	Graph::NodePtr n_4_2 = g.addNode({2, -1}, appearanceScore, disappearanceScore, 
										false, true);

	Graph::NodePtr n_4_3 = g.addNode({2, -6}, appearanceScore, disappearanceScore, 
										false, true);

	Graph::NodePtr n_4_4 = g.addNode({4, -2}, appearanceScore, disappearanceScore, 
										false, true);

	BOOST_CHECK_EQUAL(g.getNumArcs(), 31); // doesn't add disappearance moves at sink
	BOOST_CHECK_EQUAL(g.getNumNodes(), 14);

	g.addMoveArc(n_3_1, n_4_1, 0.0);
	g.addMoveArc(n_3_1, n_4_2, 0.0);
	g.addMoveArc(n_3_2, n_4_2, 0.0);
	g.addMoveArc(n_3_2, n_4_3, 0.0);
	g.addMoveArc(n_3_3, n_4_3, 0.0);
	g.addMoveArc(n_3_3, n_4_4, 0.0);
	BOOST_CHECK_EQUAL(g.getNumArcs(), 37);

	g.allowMitosis(n_3_2, n_4_3, -1);
	g.allowMitosis(n_3_3, n_4_3, 2);

	BOOST_CHECK_EQUAL(g.getNumArcs(), 39);
}

