function key = findMinCostNode_Astar(nodeList)
%FINDMINCOSTNODE_ASTAR find min f_cost A* node key in an open set

all_keys = keys(nodeList);
key = all_keys{1};

for i = 1 : length(all_keys)
    if nodeList(all_keys{i}).f_cost < nodeList(key).f_cost
        key = all_keys{i};
    end
end