function roadmap_new = updateEnclosedRoadmap(roadmap, obstacleVerts, wp_notFree, wp_update,occ)
 
% ~~~~~ connect enclosed waypoints to dummy waypoint ~~~~~
%       wp_notFree   list of waypoints not in Qfree
%       Wp_update 	list of waypoints to update   should be  2 x 2 (2 dummy
%                                                               waypoints
%       obstacleVerts       from updated polygonal world
%       occ         boolean for if the optional wall is there or not

roadmap_new = roadmap;
if ~occ             % function only does stuff if optWall is deemed not there
    
    for k = 1: size(wp_update,1)    % for each dummy waypoint
        for j = 1: size(wp_notFree,1)      % for each possible waypoint to connect to (i.e. all wps not in Qfree)
            if ~isequal(wp_update,wp_notFree(j,:))
                minE = [0 0 1000 1000 ];
                roadmap_new.E = [roadmap_new.E; minE];
                inval = 0;
                lineseg = [wp_update(k,:); wp_notFree(j,:)];
                for n = 1:length(obstacleVerts)     % for each obstacle
                    polyi = polyshape(obstacleVerts{n}(1:end-1,:));
                    [ini,~] = intersect(polyi,lineseg);
                    if ~isempty(ini)     % if intersection happens
                        inval = 1;
                        break;
                    end
                end

                if ~inval       % if edge is possible
                    possibleE = [wp_update(k,:) wp_notFree(j,:)];
                    dist = norm(wp_update(k,:)-wp_notFree(j,:));
                    if dist < norm(minE(1:2) - minE(3:4))
                        minE = possibleE;
                        roadmap_new.E(end,:) = minE;
                    end
                end
            end
        end        
    end   
end

toslice = find(ismember(roadmap_new.E, [0,0,1000,1000], 'rows'));
roadmap_new.E(toslice,:) = [];



end
