function key = findMinCostNode(nodeList)
%FINDMINCOSTNODE find min cost node key in an open set

all_keys = keys(nodeList);
key = all_keys{1};

for i = 1 : length(all_keys)
    if nodeList(all_keys{i}).cost < nodeList(key).cost
        key = all_keys{i};
    end
end