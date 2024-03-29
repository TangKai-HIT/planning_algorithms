classdef RRTstar_Planner < handle
    %RRTSTAR_PLANNER simple unidirection RRT* planner 
    %   Detailed explanation goes here

    properties
        startNode;
        goalNode;
        path; %N X dim
        pathSize = 0;
        pathLen = 0;
        goalCheckProb; %goal checking probability
        sampleRange; %2 X dim, sample range of the coordinates -- [min; max]
        extendLen; %max length of edge 
        gamma;  %rewire radius constant based on environment
        maxIter = 200; %max iteration
        calDisFcn; %distance calculation funciton handle -- input: state1, state2, output: dis
        collisDetectFcn; %line collision detection funciton handle -- input: state1, state2, output: true/false
%         useSquareDis; %use square distance flag
%         useKdTree; %use k-d tree flag
    end

    properties(Access = private)
        nodeList=[]; %all nodes on the RRT
%         kd_root; %root node on k-d tree
    end

    methods
        function obj = RRTstar_Planner(startState, goalState, extendLen, goalCheckProb, gamma, sampleRange, calDisFcn, collisDetectFcn)
            %RRT_PLANNER Construct an instance of this class
            %   Detailed explanation goes here
            if nargin>0
                obj.startNode = RRTstar_Node(startState);
                obj.goalNode = RRTstar_Node(goalState);
                obj.nodeList = [obj.nodeList, obj.startNode];
                obj.extendLen = extendLen;
                obj.gamma = gamma;
                obj.sampleRange = sampleRange;
                obj.goalCheckProb = goalCheckProb;
                obj.calDisFcn = calDisFcn;
                obj.collisDetectFcn = collisDetectFcn;
            end
        end

        function reset(obj)
            %RESET 
            obj.nodeList=[obj.startNode]; 
            obj.path = [];
            obj.pathSize = 0;
            obj.pathLen = 0;
        end

        function [path, pathLen, exitFlag] = plan(obj, maxIter)
            %PLAN start planning
            %   maxIter: max iteration;   exitFlag: 1--find goal, 0--reached max iteration
            if nargin > 0
                obj.maxIter = maxIter;
            end

            disp("start planning....");
            %construction loop
            exitFlag = 0; %exit flag
            for i = 1:obj.maxIter
                %check if goal node is connectable
                if rand < obj.goalCheckProb
                    [nearGoalNode, minDis] = obj.findNearNode(obj.goalNode.state);
                    if ~obj.collisDetectFcn(nearGoalNode.state, obj.goalNode.state)
                        obj.goalNode.parentNode = nearGoalNode;
                        obj.goalNode.cost = nearGoalNode.cost + minDis;
                        exitFlag = 1;
                        break;
                    end
                end
                %sample new state
                newState = obj.uniformSampler();
                %get nearest node
                [nearestNode, minDis] = obj.findNearNode(newState);
                %extend to limit
                [newState, ~] = obj.extendEdge(newState, nearestNode.state, minDis);
                %choose parent
                [potentParentId, edgeCosts] = obj.getPotentialParents(newState);

                %find minimum cost parent
                if ~isempty(potentParentId)
                    minCost = inf;
                    minParentId = potentParentId(1);
                    for k=1:length(potentParentId)
                        index = potentParentId(k);
                        curNode = obj.nodeList(index);
                        curCost = curNode.cost + edgeCosts(k);
                        
                        if curCost<minCost
                            minCost = curCost;
                            minParentId = index;
                        end
                    end
                    %add new node
                    newNode = RRTstar_Node(newState, obj.nodeList(minParentId), minCost);
                    obj.nodeList = [obj.nodeList, newNode];
                    %rewire step
                    rewire(obj, newNode, potentParentId, edgeCosts);
                end
            end
            
            %outPut path list if find path to goal
            if exitFlag
                fprintf("Path found after %d iterations\n", i);
                obj.getPath();
            else
                fprintf("Failed! Exit after max iterations %d\n", i);
            end
            path = obj.path;
            pathLen = obj.pathLen;
        end

        function newState = uniformSampler(obj)
            %UNIFORMSAMPLER uniform sampler
            dim = size(obj.sampleRange, 2);
            rangeSize = obj.sampleRange(2, :) - obj.sampleRange(1, :);
            newState = rand(1, dim) .* rangeSize + obj.sampleRange(1, :);
        end

        function [nearNode, minDis]= findNearNode(obj, newState)
            %FINDNEARNODE find nearest node to the new state on the tree
            nearNode = obj.nodeList(1);
            minDis = obj.calDisFcn(nearNode.state, newState);
            for i = 2:length(obj.nodeList)
                cur_node = obj.nodeList(i);
                cur_dis = obj.calDisFcn(cur_node.state, newState);
                if(cur_dis<minDis)
                    nearNode = cur_node;
                    minDis = cur_dis;
                end
            end
        end
        
        function plotTree2D(obj, ax, color, line_width)
            %PLOTTREE2D plot 2D tree
            hold(ax, "on");
            for i=1:length(obj.nodeList)
                curNode = obj.nodeList(i);
                parentNode = curNode.parentNode;
                if ~isempty(parentNode)
                    X = [curNode.state(1), parentNode.state(1)];
                    Y = [curNode.state(2), parentNode.state(2)];
                    plot(ax, X, Y, '-', Color=color, LineWidth=line_width);
                end
            end
        end

        function plotPath2D(obj, ax, color, line_width)
            %PLOTPATH2D plot 2D path
            hold(ax, "on");
            for i=2:obj.pathSize
                X = [obj.path(i, 1), obj.path(i-1, 1)];
                Y = [obj.path(i, 2), obj.path(i-1, 2)];
                plot(ax, X, Y, '-', Color=color, LineWidth=line_width);
            end
        end

    end

    methods(Access=private)
        function [newState, newDis] = extendEdge(obj, state, nearState, minDis)
            %EXTENDEDGE extend branch w.r.t extendLen
            if minDis>obj.extendLen
                newState = (state - nearState) / minDis * obj.extendLen + nearState;
                newDis = obj.extendLen;
            else
                newState = state;
                newDis = minDis;
            end
        end
        
        function [potentParentId, edgeCosts] = getPotentialParents(obj, newState)
            %GETPOTENTIALPARENTS get potential parents' index
            numNodes = length(obj.nodeList) + 1;
            dim = length(newState);
            radius = obj.gamma * power(log(numNodes) / numNodes, 1/dim); %rewire redius
            radius = min(radius, obj.extendLen);
            
            %search potential parents
            potentParentId = [];
            edgeCosts = [];
            for i=1:length(obj.nodeList)
                curNode = obj.nodeList(i);
                curEdgeCost = obj.calDisFcn(newState, curNode.state);
                if curEdgeCost<=(radius+1e-6) && ~obj.collisDetectFcn(curNode.state, newState)
                    potentParentId = [potentParentId, i];
                    edgeCosts = [edgeCosts, curEdgeCost];
                end
            end
        end

        function rewire(obj, newNode, potentParentId, parentEdgeCosts)
            %REWIRE rewire step of RRT*
            for i = 1: length(potentParentId)
                neighborNode = obj.nodeList(potentParentId(i));
                potential_cost = parentEdgeCosts(i) + newNode.cost;
                
                if potential_cost<neighborNode.cost %rewire
                    neighborNode.parentNode = newNode;
                    cost_modify = potential_cost - neighborNode.cost;
                    neighborNode.cost = potential_cost;
                    obj.costModifyRecursion(cost_modify, neighborNode);
                end
            end
        end

        function costModifyRecursion(obj, cost_modify, parentNode)
            %costModifyRecursion modify cost of a node to its leaves recursively
            for node = obj.nodeList
                if(~isempty(node.parentNode) && node.parentNode==parentNode)
                    node.cost = node.cost + cost_modify;
                    obj.costModifyRecursion(cost_modify, node);
                end
            end
        end

        function getPath(obj)
            %GETPATH output path points to obj.path
            curNode = obj.goalNode;
            obj.path = [];
            while(~isempty(curNode.parentNode))
                obj.path = [obj.path; curNode.state];
                curNode = curNode.parentNode;
            end
            obj.path = [obj.path; curNode.state];
            obj.path = flip(obj.path);
            obj.pathSize = size(obj.path, 1);
            obj.pathLen = obj.goalNode.cost;
        end
    end

end