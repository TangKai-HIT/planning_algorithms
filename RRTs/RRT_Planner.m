classdef RRT_Planner < handle
    %RRT_PLANNER simple unidirection RRT planner 
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
        function obj = RRT_Planner(startState, goalState, extendLen, goalCheckProb, sampleRange, calDisFcn, collisDetectFcn)
            %RRT_PLANNER Construct an instance of this class
            %   Detailed explanation goes here
            if nargin>0
                obj.startNode = RRT_Node(startState);
                obj.goalNode = RRT_Node(goalState);
                obj.nodeList = [obj.nodeList, obj.startNode];
                obj.extendLen = extendLen;
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
                        obj.goalNode.lenToParent = minDis;
                        exitFlag = 1;
                        break;
                    end
                end
                %sample new state
                newState = obj.uniformSampler();
                %get nearest node
                [nearestNode, minDis] = obj.findNearNode(newState);
                %extend to limit
                [newState, newDis] = obj.extendEdge(newState, nearestNode.state, minDis);
                %check line collision
                if ~obj.collisDetectFcn(nearestNode.state, newState)
                    %add new node
                    newNode = RRT_Node(newState, nearestNode, newDis);
                    obj.nodeList = [obj.nodeList, newNode];
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
        
        function getPath(obj)
            %GETPATH output path points to obj.path
            curNode = obj.goalNode;
            obj.path = [];
            while(~isempty(curNode.parentNode))
                obj.path = [obj.path; curNode.state];
                obj.pathLen = obj.pathLen + curNode.lenToParent;
                curNode = curNode.parentNode;
            end
            obj.path = [obj.path; curNode.state];
            obj.path = flip(obj.path);
            obj.pathSize = size(obj.path, 1);
        end
    end

end