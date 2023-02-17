classdef AstarNode < handle
    %ASTARNODE node in A* algorithm
    %   Detailed explanation goes here

    properties
        coordinate;
        g_cost; % actual current cost from start point 
        h_cost; % estimated heuristic cost from current point to goal point
        f_cost; % total estimated cost from current point to goal point: f(x) = g(x) + h(x)
        parentIndex;
    end

    methods
        function obj = AstarNode(coordinate, g_cost, h_cost, parentIndex)
            %ASTARNODE Construct an instance of this class
            %   Detailed explanation goes here
            if nargin <1
                obj.g_cost = inf;
                obj.f_cost = inf;
            elseif nargin<2
                obj.coordinate = coordinate;
                obj.g_cost = inf;
                obj.f_cost = inf;
            elseif nargin<4
                obj.g_cost = g_cost;
                obj.h_cost = h_cost;
                obj.f_cost = g_cost + h_cost;
                obj.coordinate = coordinate;
            else
                obj.g_cost = g_cost;
                obj.h_cost = h_cost;
                obj.f_cost = g_cost + h_cost;
                obj.coordinate = coordinate;
                obj.parentIndex = parentIndex;
            end
        end
       
    end
end