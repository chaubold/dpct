# run this as test? https://cmake.org/pipermail/cmake/2010-August/039174.html
import dpct

weights = {"weights": [10, 10, 10, 500, 500]}

graph = {
	"settings" : {
		"statesShareWeights" : True
	},

	"segmentationHypotheses" : [
		{ 
			"id" : 2, 
			"features" : [[1.0], [0.0]], 
			"divisionFeatures" : [[0.0], [-5.0]],  
			"appearanceFeatures" : [[0], [0]], 
			"disappearanceFeatures" : [[0], [50]], 
			"timestep" : [1,1]
		},
		{ "id" : 3, "timestep" : [1,1], "features" : [[1.0], [0.0]], "divisionFeatures" : [[0], [-5]], "appearanceFeatures" : [[0], [0]], "disappearanceFeatures" : [[0], [50]]},
		{ "id" : 4, "timestep" : [2,2], "features" : [[1.0], [0.0]], "appearanceFeatures" : [[0], [50]], "disappearanceFeatures" : [[0], [-2]]},
		{ "id" : 5, "timestep" : [2,2], "features" : [[1.0], [0.0]], "appearanceFeatures" : [[0], [50]], "disappearanceFeatures" : [[0], [-2]]},
		{ "id" : 6, "timestep" : [2,2], "features" : [[1.0], [0.0]], "appearanceFeatures" : [[0], [50]], "disappearanceFeatures" : [[0], [-4]]}
	],

	"linkingHypotheses" : [
		{ "src" : 2, "dest" : 4, "features" : [[0], [-4]]},
		{ "src" : 2, "dest" : 5, "features" : [[0], [-3]]},
		{ "src" : 3, "dest" : 5, "features" : [[0], [-1]]},
		{ "src" : 3, "dest" : 6, "features" : [[0], [-4]]}
	]
}

res = dpct.trackFlowBased(graph, weights)

expectedResult = {'detectionResults': [{'id': 2, 'value': 1},
  {'id': 3, 'value': 1},
  {'id': 4, 'value': 1},
  {'id': 5, 'value': 1},
  {'id': 6, 'value': 1}],
 'divisionResults': [{'id': 2, 'value': True},
  {'id': 3, 'value': False},
  {'id': 4, 'value': False},
  {'id': 5, 'value': False},
  {'id': 6, 'value': False}],
 'linkingResults': [{'dest': 4, 'src': 2, 'value': 1},
  {'dest': 5, 'src': 2, 'value': 1},
  {'dest': 5, 'src': 3, 'value': 0},
  {'dest': 6, 'src': 3, 'value': 1}]}

assert(res == expectedResult)