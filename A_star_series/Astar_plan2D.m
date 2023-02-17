function pathNodeList = Astar_plan2D(start_state, goal_state, map, action_set, heuristicFcn)
%ASTAR_PLAN2D use A* for planning on 2D map
%   input:
%       start_state, goal_state: coordinate of start, goal state on map
%                                                  table (range: [1, size(map,1)],  [1, size(map,2)])
%       map: discrete map with obstacle (2d-array)
%       action_set: struct, members -- actions(cell), cost(array) [note: fixed cost for each action]
%       heuristicFcn: function handle of getting heuristic cost from current state to goal -- h(x)
%   output:
%       pathNodeList: AstarNode array

%% Init
openList = containers.Map('KeyType', 'int32', 'ValueType', 'any'); %point index set of those haven't been accepted
closedList = containers.Map('KeyType', 'int32', 'ValueType', 'any'); %point index set of those have been accepted

x_width = size(map, 1) + 1;
x_max = size(map, 1);
y_max = size(map, 2);
x_min = 1;
y_min = 1;

startNode = AstarNode(start_state, 0, heuristicFcn(start_state, goal_state)); %start node: index 1
goalNode = AstarNode(goal_state); %goal node: index 2

startKey = calKeyIndex2D(start_state, x_width, x_min, y_min);
goalKey = calKeyIndex2D(goal_state, x_width, x_min, y_min);
openList(startKey) = startNode;
openList(goalKey) = goalNode;

%% Explore & find shortest path
findFlag = false;
while true
    %if no way to reach goal (open list empty)
    if isempty(openList)
        disp("No way to reach goal!");
        break;
    end

    %find min cost value point in open list
    cur_key = findMinCostNode_Astar(openList);
    cur_Node = openList(cur_key);
    
    %accept it in closed list & remove it from open list
    closedList(cur_key) = cur_Node;
    remove(openList, cur_key);

    %end-condition: find goal
    if cur_key == goalKey
        findFlag = true;
        break;
    end

    %traverse neighbors
    for i = 1:length(action_set.actions)
        %neighbors' coordinates
        neigh_move = action_set.actions{i};
        neigh_coordinate = cur_Node.coordinate +neigh_move;

        %check if node feasible(not out of boundary or in obstacle)
        if neigh_coordinate(1)<x_min || neigh_coordinate(1)>x_max || ...
                neigh_coordinate(2)<y_min || neigh_coordinate(2)>y_max || ...
                    map(neigh_coordinate(1), neigh_coordinate(2)) == 0
            continue;
        end

        %get neighbor cost
        neigh_cost = action_set.cost(i);
        cur_gcost = neigh_cost + cur_Node.g_cost; %current g_cost

        %check if new node
        neigh_keyIndex = calKeyIndex2D(neigh_coordinate, x_width, x_min, y_min);
        
        isOpenList = isKey(openList, neigh_keyIndex);
        isClosedList = isKey(closedList, neigh_keyIndex);

        if isOpenList || isClosedList %already in list
            if isOpenList %in open list
                neighNode = openList(neigh_keyIndex); %get neighboor node handle
            else %in closed list
                neighNode = closedList(neigh_keyIndex); %get neighboor node handle
            end
            %update cost & parent index
            if neighNode.g_cost > cur_gcost
                neighNode.g_cost =  cur_gcost; 
                neighNode.f_cost = neighNode.g_cost + neighNode.h_cost;
                neighNode.parentIndex = cur_key;
            end
        else % new node
            cur_hcost = heuristicFcn(neigh_coordinate, goal_state); %h_cost
            newNode = AstarNode(neigh_coordinate , cur_gcost, cur_hcost, cur_key);
            openList(neigh_keyIndex) = newNode; %add new node to open list
        end
    end
end

%% Get shortest path node list
if findFlag
    cur_key = goalKey;
    id = 1;
    while true
        pathNodeList(id) = closedList(cur_key);
    
        if cur_key == startKey
            break
        end
    
        cur_key = pathNodeList(id).parentIndex;
        id = id + 1;
    end

    pathNodeList = flip(pathNodeList); %reverse index

else
    pathNodeList = [];
end