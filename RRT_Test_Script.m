clc;

% params for function [vertices, edges, path] = rrt(map, q_start, q_goal, k, delta_q, p)

% MAP
% map = load('map.mat');
% q_start = [80, 70];
% q_goal =  [707, 615];

% MAZE
map = load('maze.mat');
q_start = [206, 198];
q_goal = [416, 612];

map = map.map;

k = 10000;
delta_q = 50;
p = 0.3;

% params for function [path_smooth] = smooth(map, path, vertices, delta)
delta = 5;


[vertices, edges, path] = rrt(map, q_start, q_goal, k, delta_q, p);

path_smooth = smooth(map, path, vertices, delta);


    imshow(int32(1 - map), []);
    title('RRT (Rapidly-Exploring Random Trees) - Smooth');
    % imagesc(1 - map);
    % colormap(gray);
    
    hold on;
    
    [edgesRowCount, ~] = size(edges);
    
    for ii = 1 : edgesRowCount
        plot(vertices(ii, 1), vertices(ii, 2), 'cyan*', 'linewidth', 1);
        plot([vertices(edges(ii, 1), 1), vertices(edges(ii, 2), 1)], ...
        [vertices(edges(ii, 1), 2), vertices(edges(ii, 2), 2)], ...
         'b', 'LineWidth', 1);
    end
    
    plot(q_start(1), q_start(2), 'g*', 'linewidth', 1);
    plot(q_goal(1), q_goal(2), 'r*', 'linewidth', 1);
    
    
    [~, pathCount] = size(path);
    
    for ii = 1 : pathCount - 1
        %plot(vertices(ii, 1), vertices(ii, 2), 'cyan*', 'linewidth', 1);
        plot([vertices(path(ii), 1), vertices(path(ii + 1), 1)], ...
        [vertices(path(ii), 2), vertices(path(ii + 1), 2)], ...
         'r', 'LineWidth', 1);
    end
    
    [~, pathCount] = size(path_smooth);
    
    for ii = 1 : pathCount - 1
        %plot(vertices(ii, 1), vertices(ii, 2), 'cyan*', 'linewidth', 1);
        plot([vertices(path_smooth(ii), 1), vertices(path_smooth(ii + 1), 1)], ...
        [vertices(path_smooth(ii), 2), vertices(path_smooth(ii + 1), 2)], ...
         'black', 'LineWidth', 2);
    end