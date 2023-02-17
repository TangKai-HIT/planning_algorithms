%demo1: dijkstra in cluttered 2D environment 
addpath(genpath("../"));

%% Create environment
% basics
bbox = [0 1 0 1]; %unit bounding box
resolution = 0.01; %map resolution
% obstacle shape
vertical_disp = [0.4 0.7 0.2 0.5 1.0];
wall_width = 0.02;
gap_clearance = 0.1;
rectangle_array = get_left_right_tunnel( bbox, vertical_disp, wall_width, gap_clearance);
% convert to map
map = convert_rectangle_shape_array_to_map( rectangle_array, bbox, resolution ); %coordinate (x,y) = map.table(x+1, y+1)
% show map
figure(Position=[300, 100, 800, 800]);
visualize_map(map);
axis equal
axis(bbox)

%% Define start, goal & action set
start = [0.1, 0.1] / resolution + 1;
goal = [0.9, 0.9] / resolution + 1;
action_set.actions = {[1, 1], [-1, -1], [-1, 1], [1, -1], ... 
                                        [0, 1], [1, 0], [0, -1], [-1, 0]};
action_set.cost = zeros(1, 8);
action_set.cost(1:4) = sqrt(2);
action_set.cost(5:8) = 1;

%% Start planning
disp("Start dijktra algorithm ...");
tic;
pathNodeList = dijkstra_plan2D(start, goal, map.table, action_set);
solve_time = toc;
fprintf("Solve time: %.3f s\n", solve_time);

%% Show result plot
for node = pathNodeList
    map.table(node.coordinate(1), node.coordinate(2)) = 0.5;
end

visualize_map(map);
axis equal
axis(bbox)
