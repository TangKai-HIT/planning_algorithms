function [path, pathSize, pathLen] = getPathRRT(endNode)
%GETPATHRRT get path coordinate list and length given an end node of RRT

curNode = endNode;
path = [];
pathLen = 0;
while(~isempty(curNode.parentNode))
    path = [path; curNode.state];
    pathLen = pathLen + curNode.lenToParent;
    curNode = curNode.parentNode;
end
path = [path; curNode.state];
path = flip(path);
pathSize = size(path, 1);
end