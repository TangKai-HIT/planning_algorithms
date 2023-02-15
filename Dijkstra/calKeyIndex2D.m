function keyIndex = calKeyIndex2D(coordinate, x_size, x_min, y_min)
%CALKEYINDEX2D calculate map node key for the mapping container of points with 2D coordinate

keyIndex = x_size * (coordinate(2) - y_min) + (coordinate(1) - x_min);