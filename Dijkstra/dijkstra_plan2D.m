function pathNodeList = dijkstra_plan2D(start_state, goal_state, map, action_set)
%DIJKSTRA_PLAN2D use dijkstra for planning 
%   input:
%       start_state, goal_state: coordinate of start, goal state on map
%                                                  table (range: [1, size(map,1)],  [1, size(map,2)])
%       map: discrete map with obstacle (2d-array)
%       action_set: struct, members -- actions(cell), cost(array)
%   output:
%       pathNodeList: dijkNode array

%% Init
openList = containers.Map('KeyType', 'int32', 'ValueType', 'any'); %point index set of those haven't been accepted
closedList = containers.Map('KeyType', 'int32', 'ValueType', 'any'); %point index set of those have been accepted

x_width = size(map, 1) + 1;
x_max = size(map, 1);
y_max = size(map, 2);
x_min = 1;
y_min = 1;

startNode = dijkNode(start_state, 0); %start node: index 1
goalNode = dijkNode(goal_state); %goal node: index 2

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
    cur_key = findMinCostNode(openList);
    cur_Node = openList(cur_key);
    
    %accept it in closed list & remove it from open list
    closedList(cur_key) = cur_Node;
    remove(openList, cur_key);

    %end-condition: find goal
    if cur_key == goalKey
        findFlag = true;
        disp("Find goal!");
        break;
    end

    %traverse neighbors
    for i = 1:length(action_set.actions)
        %neighbors' coordinates
        neigh_move = action_set.actions{i};
        neigh_cost = action_set.cost(i);
        neigh_coordinate = cur_Node.coordinate +neigh_move;

        %check if node feasible(not out of boundary or in obstacle)
        if neigh_coordinate(1)<x_min || neigh_coordinate(1)>x_max || ...
                neigh_coordinate(2)<y_min || neigh_coordinate(2)>y_max || ...
                    map(neigh_coordinate(1), neigh_coordinate(2)) == 0
            continue;
        end

        %check if new node
        neigh_keyIndex = calKeyIndex2D(neigh_coordinate, x_width, x_min, y_min);
        
        isOpenList = isKey(openList, neigh_keyIndex);
        isClosedList = isKey(closedList, neigh_keyIndex);

        if isOpenList || isClosedList %already in list
            cur_cost = neigh_cost + cur_Node.cost;
            if isOpenList %in open list
                neighNode = openList(neigh_keyIndex); %get neighboor node handle
            else %in closed list
                neighNode = closedList(neigh_keyIndex); %get neighboor node handle
            end
            %update cost & parent index
            if neighNode.cost > cur_cost
                neighNode.cost =  cur_cost; 
                neighNode.parentIndex = cur_key;
            end
        else
            newNode = dijkNode(neigh_coordinate , neigh_cost + cur_Node.cost, cur_key);
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