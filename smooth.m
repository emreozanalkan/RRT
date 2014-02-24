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


path_smooth = [0];


end

function [isBelongsFreeSpace] = isEdgeQNearQNewBelongsFreeSpace(map, q_near, q_new)
    
    % In order to check if an edge belongs to the free space,
    % use the incremental (left) or subdivision (right) strategies.
    % You can use 10 intermediate points.
    intermediatePointCount = 10;
    
    v = double(q_new - q_near);
    
    distance = norm(v);
    
    u = v / distance;
    
    delta_q = distance / intermediatePointCount;
    
    currentCoordinate = double(q_near);
    
    for ii = 1 : intermediatePointCount
        
        currentCoordinate = currentCoordinate + (delta_q * u);
        
        if map(int32(currentCoordinate(2)), int32(currentCoordinate(1))) == 1 % map(q_new(1), q_new(2))
            isBelongsFreeSpace = 0;
            return;
        end
        
    end
    
    isBelongsFreeSpace = 1;

end

% http://math.stackexchange.com/questions/175896/finding-a-point-along-a-line-a-certain-distance-away-from-another-point
% http://stackoverflow.com/questions/1061276/how-to-normalize-a-vector-in-matlab-efficiently-any-related-built-in-function
function [q_new] = findQNew(q_near, q_rand, delta_q)

    v = double(q_rand - q_near);
    
    u = v / norm(v);
    
    q_new = int32(double(q_near) + delta_q * u);
    
end