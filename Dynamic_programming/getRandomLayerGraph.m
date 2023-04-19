function [startNode, numNodes] = getRandomLayerGraph(numMiddleLayers, numNodesPerLayer, randRange, seed)
%GETRANDOMLAYERGRAPH  Construct a random directional weighted layer-graph
%   此处显示详细说明

rng(seed); %seed
randWidth = randRange(2) - randRange(1);

NodeQueue = []; %node queue for construction 

numNodes = 1; %node index (coordinate)
startNode = dirctedNode(numNodes); %start node
startNode.cost = 0; %init cost

NodeQueue = [NodeQueue, startNode];

% startNode->middle layers
for i=1:numMiddleLayers
    curSize = length(NodeQueue);
    
    %add a layer of new nodes 
    curChildren = [];
    for j=1:numNodesPerLayer
        numNodes = numNodes+1;
        newNode = dirctedNode(numNodes);
    
        NodeQueue = [NodeQueue, newNode];
        curChildren = [curChildren, newNode];
    end

    %traverse each layer
    for k = 1:curSize
        curNode = NodeQueue(1);
        NodeQueue(1) = []; %pop front

        %set weights
        curNode.weights = floor(randWidth*rand(1, numNodesPerLayer)+randRange(1));
        curNode.children = curChildren;
    end
end

% middle layers -> end layer
numNodes = numNodes+1;
endNode = dirctedNode(numNodes);
while ~isempty(NodeQueue) 
    curNode = NodeQueue(1);
    NodeQueue(1) = []; %pop front
    curNode.weights = floor(randWidth*rand(1)+randRange(1));
    curNode.children = endNode;
end

end

