function wp_notFree = checkWPinFree(wp_tocheck, obstacleVerts)
% Stores which waypoints (waypoints + EC waypoints) are not in Qfree, which means that they are in an enclosed area
% 
% INPUTS:
% 	wp_tocheck		[waypoints; ECwaypoints]
% 	obstacleVerts 		from polygonalWorld
% OUTPUTS:
% 	wp_notFree 		list of waypoints not in Qfree
%     
    wp_notFree = [];     % waypoints not in Qfree
    for i = 1:size(wp_tocheck,1)
        for n = 1:length(obstacleVerts)    % for each obst
            check = inpolygon(wp_tocheck(i,1), wp_tocheck(i,2), obstacleVerts{n}(1:end-1,1),obstacleVerts{n}(1:end-1,2));
            if check    % if in obstacle, not in Qfree
                in_Qfree = 0;
                wp_notFree = [wp_notFree; wp_tocheck(i,:)];
                break;
            else
                in_Qfree = 1;
            end
        end
    end



end