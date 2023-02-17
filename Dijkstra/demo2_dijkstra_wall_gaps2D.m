%demo2: dijkstra in 2D environment of walls with gaps
addpath(genpath("../"));

%% Create environment
% basics
bbox = [0 1 0 1]; %unit bounding box
resolution = 0.01; %map resolution
% obstacle shape
rectangle_array1 = get_wall_with_uniform_gaps( bbox, 0.3, 0.2, 0.1, [1 1 0 1] );
rectangle_array2 = get_wall_with_uniform_gaps( bbox, 0.7, 0.2, 0.1, [1 0 1 1] );
rectangle_array = [rectangle_array1 rectangle_array2];
% convert to map
map = convert_rectangle_shape_array_to_map( rectangle_array, bbox, resolution ); %coordinate (x,y) = map.table(x+1, y+1)
% show map
figure(Position=[300, 100, 800, 800]);
visualize_map(map);
axis equal
axis(bbox)

%% Define start, goal & action set
start = [0.1, 0.5] / resolution + 1;
goal = [0.9, 0.6] / resolution + 1;
action_set.actions = {[1, 1], [-1, -1], [-1, 1], [1, -1], ... 
                                        [0, 1], [1, 0], [0, -1], [-1, 0]};
action_set.cost = zeros(1, 8);
action_set.cost(1:4) = sqrt(2);
action_set.cost(5:8) = 1;

%% Start planning
disp("Start dijkstra algorithm ...");
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
