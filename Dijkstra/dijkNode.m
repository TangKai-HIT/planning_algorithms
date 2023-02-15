classdef dijkNode < handle
    %DIJKNODE node in dijkstra algorithm
    %   Detailed explanation goes here

    properties
        coordinate;
        cost;
        parentIndex;
    end

    methods
        function obj = dijkNode(coordinate, cost, parentIndex)
            %DIJKNODE Construct an instance of this class
            %   Detailed explanation goes here
            if nargin <1
                obj.cost = inf;
            elseif nargin<2
                obj.coordinate = coordinate;
                obj.cost = inf;
            elseif nargin<3
                obj.cost = cost;
                obj.coordinate = coordinate;
            else
                obj.cost = cost;
                obj.coordinate = coordinate;
                obj.parentIndex = parentIndex;
            end
        end
       
    end
end