function [dist,path] = findPath(roadmap,q_start,q_goal)
%   given a polygonal environment, a roadmap, and initial and goal
%   points, returns the shortest path connecting the initial and goal points
%   shortest path is given as a series of coords [nx2]

    % add IDs to nodes and edges
	V = [ (1:size(roadmap.V,1))'  roadmap.V];
    vID = V(:,1);
    
    E = zeros(size(roadmap.E,1),2);
    
    % for each edge, find corresponding vertex IDs
    for i = 1:size(roadmap.E,1)
        N = zeros(1,2);
        c = 0;
        for j = [1,3]
            c = c+1;
            coord1 = [roadmap.E(i,j), roadmap.E(i,j+1)];
            coord2 = [roadmap.E(i,j+1), roadmap.E(i,j)];
            idx = find(ismember(roadmap.V,coord1,'rows'),1);
            if isempty(idx)
                idx = find(ismember(roadmap.V,coord2,'rows'),1);
            end
            if isempty(idx)
               idx 
            end
            N(c) = vID(idx);
        end
        E(i,:) = N;
    end
    E = [ (1:size(roadmap.E,1))'  E];
    
    

    % find q_start and q_goal IDs
    startID = find(ismember(roadmap.V, q_start, 'rows'),1);
    goalID = find(ismember(roadmap.V, q_goal, 'rows'),1);

    

    % find shortest path (path only contains IDs)
    [dist,pathN] = dijkstra(V, E,startID,goalID);
    
    if isnan(pathN)     % no possible path
        dist = NaN; path = [];
    else
    
        % convert IDs in pathN to actual xy coords
        path = zeros(length(pathN),2);
        for i = 1:length(pathN)
            path(i,:) = V(pathN(i),2:end);
        end
    end
end
