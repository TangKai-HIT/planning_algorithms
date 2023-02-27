%demo1: RRT planning in 2D environment with poisson forest
addpath(genpath("../"));
clc;
clear;
close all;

%% Create environment
bbox = [0 1 0 1]; %unit bounding box
side = 0.1;
num_squares = 20; % number of rectangles

rng(1); %random seed
square_array = get_square_poisson_forest( bbox, side, num_squares );

resolution = 0.001; %resolution of map
map = convert_rectangle_shape_array_to_map( square_array, bbox, resolution );

%% Set start , goal
origin = [0, 0];
start = [0.05, 0.05];
goal = [0.8, 0.9];

%% Visualize environment
figure(Position=[300, 100, 800, 800]);
visualize_map(map);
%plot start, goal
hold("on");
plot(gca, start(1),  start(2), 'xr', MarkerSize=18);
plot(gca, goal(1),  goal(2), 'xr', MarkerSize=18);

%% Setup RRT planner
%setting parameters
extendLen = 0.5;
goalCheckProb = 0.5;

xSampleRange = [0; 1];
ySampleRange = [0; 1];
sampleRange = [xSampleRange, ySampleRange];

calDisFcn = @getDistanceL2;
collisDetectFcn = @(state1, state2) lineCollisCheck_map(state1, state2, map);

maxIter = 300;
%create planner
myRRT = RRT_Planner(start, goal, extendLen, goalCheckProb, sampleRange, calDisFcn, collisDetectFcn);

%% Start Planning
tic;
[path, pathLen, exitFlag] = myRRT.plan(maxIter);
solve_time = toc;
fprintf("Solve time: %.3f s\n", solve_time);

%% Show result plot
myRRT.plotTree2D(gca, 'g', 1);
myRRT.plotPath2D(gca, 'r', 1.5);