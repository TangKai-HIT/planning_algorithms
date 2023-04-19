classdef dirctedNode < handle
    %DIRCTEDNODE node in dircted weighted graph stored by list
    %   Detailed explanation goes here

    properties
        coordinate;
        cost = inf; %min cost
        minCostParent; % min cost Parent
        weights; %weights on edge to children
        children;
    end

    methods
        function obj = dirctedNode(coordinate, weights, children)
            %DIJKNODE Construct an instance of this class
            %   Detailed explanation goes here
            if nargin <1
                obj.cost = inf;
            elseif nargin<2
                obj.coordinate = coordinate;
            elseif nargin<3
                obj.weights = weights;
                obj.coordinate = coordinate;
            else
                obj.coordinate = coordinate;
                obj.weights = weights;
                obj.children = children;
            end
        end
       
    end
end