function [pathNodesList, minCost] = DP_directedLayerGraph(startNode)
%DP_DIRECTEDLAYERGRAPH dynamic programming on point-to-point directed weighted layer-graph
%   Detailed explanation goes here

pathNodesList = [];

%% Value iteration
NodeQueue = []; %queue for layer traverse

NodeQueue = [NodeQueue, startNode];

%endNode =[];

while ~isempty(NodeQueue)
    curSize = length(NodeQueue);

    %traverse each layer
    for k = 1:curSize
        curNode = NodeQueue(1);
        NodeQueue(1) = []; %pop front
        
        if isempty(curNode.children) %end node
            endNode = curNode;
            break;
        end

        %update children node
        for j = 1:length(curNode.children)
            curCost = curNode.cost + curNode.weights(j);

            if curCost < curNode.children(j).cost %update min cost and parent
                curNode.children(j).cost = curCost;
                curNode.children(j).minCostParent = curNode;
            end
        end

        %add next layer in queue (layer--suppose fully connected)
        if k==1
            NodeQueue = [NodeQueue, curNode.children];
        end
    end
end

%% Get node path & min cost
minCost = endNode.cost;

curNode = endNode;
pathNodesList = [pathNodesList, endNode];

while ~isempty(curNode.minCostParent)
    curNode = curNode.minCostParent;
    pathNodesList = [curNode, pathNodesList];
end