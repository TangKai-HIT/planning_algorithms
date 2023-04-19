%demo: DP on directional connected weighted layer-graph
addpath(genpath("../"));

%% Construct a random directional weighted layer-graph
middleLayers = 5;
numPerLayer = 5;
randomSeed = 2;
weightRange =  [1,10];

[startNode, numNodes] = getRandomLayerGraph(middleLayers, numPerLayer, weightRange, randomSeed);

%% DP get min cost
[pathNodesList, minCost] = DP_directedLayerGraph(startNode);
