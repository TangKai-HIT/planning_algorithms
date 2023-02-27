classdef RRT_Node < handle
    %RRT_Node node in a RRT
    %   Detailed explanation goes here

    properties
        state
        parentNode %parent node on RRT
        lenToParent %edge length to parent node
%         kd_childNodes %children nodes on k-d tree
    end

    methods
        function obj = RRT_Node(state, parentNode, lenToParent)
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
                obj.lenToParent = lenToParent;
            end
        end

    end
end