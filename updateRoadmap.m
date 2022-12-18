function roadmap_new = updateRoadmap(roadmap, wp_update,occ, obstacleVerts)
% given an existing roadmap and waypoints to update, update the roadmap (remove vertex and edges out of vertex
% 
% INPUTS
%       Roadmap     roadmap struct with 
%                       roadmap.V       n x 2 vertices [x y]  
%                       roadmap.E       m x 4 edges [x1 y1 x2 y2
%       Wp_update 	list of waypoints to update   should be  2 x 2 (2 dummy
%                                                               waypoints
%       occ         boolean for if the optional wall is there or not
% 
% OUTPUTS
%       roadmap_new     Updated roadmap with same structure as roadmap

roadmap_new = roadmap;
% idxListV = zeros(size(wp_update,2),1);   % indices where each wp is at
% idxListE = [];

if occ       % if wall is occupied: remove V and E
%     UNCOMMENT THESE LINES IF ROADMAP CONTAINS DUMMY WP

%     for i = 1:size(wp_update,2)    % for each waypoint
%         wp = wp_update(i,:);
%         idxListV(i) = find(ismember(roadmap.V, wp, 'rows'),1);
%         idxListE1 = find(ismember(roadmap.E(:,1:2), wp, 'rows'));
%         if isempty(idxListE1)
%             idxListE1 = find(ismember(roadmap.E(:,3:4), wp, 'rows'),1);
%         end
% 
%         idxListE = [idxListE; idxListE1];
%     end

%     roadmap_new.V(idxListV,:) = [];   % remove waypoints from vertices
%     roadmap_new.E(idxListE,:) = [];   % remove edges

else        % if wall is unoccupied: connect the two dummy WP and to its closest neighboring vertex 
    newE = [wp_update(1,1:2) wp_update(2,1:2) ];
    roadmap_new.E = [roadmap_new.E; newE];
    
    
    for i = 1:size( wp_update,1)   % for each dummy wp
        minE = [0 0 1000 1000 ];
        for j = 1: size(roadmap.V,1)      % for each possible waypoint to connect to (i.e. all wps)
            inval = 0;
            lineseg = [wp_update(i,:); roadmap.V(j,:)];
            for n = 1:length(obstacleVerts)     % for each obstacle
                polyi = polyshape(obstacleVerts{n}(1:end-1,:));
                [ini,~] = intersect(polyi,lineseg);
                if ~isempty(ini)     % if intersection happens
                    inval = 1;
                    break;
                end
            end
            
            if ~inval       % if edge is possible
                possibleE = [wp_update(i,:) roadmap.V(j,:)];
                dist = norm(wp_update(i,:)-roadmap.V(j,:));
                if dist < norm(minE(1:2) - minE(3:4))
                    minE = possibleE;
                end
            end
        end
        
        roadmap_new.E = [roadmap_new.E; minE];    
    end

end
    % add dummy wp to vertices
    roadmap_new.V = [roadmap_new.V; wp_update(1,1:2); wp_update(2,1:2)];

    

toslice = find(ismember(roadmap_new.E, [0,0,1000,1000], 'rows'));
roadmap_new.E(toslice,:) = [];


end