classdef RRTstar_Node < handle
    %RRTSTAR_NODE node in a RRT
    %   Detailed explanation goes here

    properties
        state
        parentNode %parent node on RRT
        cost %total cost from start node
%         kd_childNodes %children nodes on k-d tree
    end

    methods
        function obj = RRTstar_Node(state, parentNode, cost)
            %RRT_NODE Construct an instance of this class
            %   Detailed explanation goes here
            if nargin == 1
                obj.state = state;
            elseif nargin == 2
                obj.state = state;
                obj.parentNode = parentNode;
            elseif nargin == 3
                obj.state = state;
                obj.parentNode = parentNode;
                obj.cost = cost;
            end
        end

    end
end