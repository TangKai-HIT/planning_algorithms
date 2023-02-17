function h = getHeuristicCost2D(curState, goalState, weight)
%GETHEURISTICCOST2D get heuristic cost from current state to goal in equally spaced 2D

x_steps = abs(curState(1) - goalState(1));
y_steps = abs(curState(2) - goalState(2));

if x_steps<=y_steps
    h = sqrt(2) * x_steps + (y_steps - x_steps);
else
    h = sqrt(2) * y_steps + (x_steps - y_steps);
end

h = h * weight;