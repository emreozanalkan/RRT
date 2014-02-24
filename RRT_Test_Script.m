clc;

map = load('map.mat');

map = map.map;

smoothingDelta = 5;

[vertices, edges, path] = rrt(map, [80, 70], [707, 615], 10000, 50, 0.3);

path_smooth = smooth(map, path, vertices, smoothingDelta);