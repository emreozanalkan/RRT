function [path_smooth] = smooth(map, path, vertices, delta)
%SMOOTH Smoothing algorithm for obtaining a shorter and less noisy path.
%   We will use the greedy approach: Connect q_goal from q_start,
%   if it fails try from a closer position until it connects.
%   Once q_goal is connected, start again with its directly connected position.
%
% map: matrix that you can obtain loading the ?.mat? files.
%
% path: list of vertex indices from the start vertex (q_start) to the goal vertex (q_goal).
% The list MUST be represented as a row vector.
%
% vertices: list of x and y coordinates of the vertices.
% The first vertex will correspond to the start position and the last one will correspond t
% the goal position. The variable MUST have 2 columns for x and y coordinates and
% n rows (being n the number of vertices found in the tree).
%
% delta: incremental distance that will be used to check if direct connection between
% the vertices of the path is inside the free space. The edges will be divided obtaining
% several points, each of them separated this delta distance.
%
% path_smooth: reduced list of vertex indices from the start vertex (q_start) to the goal vertex
% (q_goal) after applying the smoothing algorithm. The list MUST be represented as a row vector.





end

