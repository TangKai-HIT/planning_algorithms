function dis = getDistanceL2(state1, state2)
%GETDISTANCEL2 get L2 distance in Euclidean space

dis = norm(state1 - state2);