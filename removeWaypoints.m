function [q_wpNew, wp_removed] = removeWaypoints(q_wp, optWall, optWallsStruct)
% given an optWall that we?ve determined is not there, remove the corresponding dummy waypoints from q_wp (list of qgoal) 
% INPUTS:
% 	Q_wp 			list of all current waypoints
% 	optWall 		wall to remove
% 	optWallsStruct		optional wall structure where
% 				optWallsStruct.walls		same as [id optWall]  nx5
% 				optWallsStruct.wp		corresponding list of dummy wps 
% size 2*n x 2??
% OUTPUTS: 
% Q_wpNew		updated q_wp
% wp removed        waypoints that are removed from qgoal


% Find which row index corresponds to optWall in optWallsStruct.walls
idx = find(ismember(optWallsStruct.walls(:,2:end), optWall, 'rows'),1);

% the 2 dummy waypoints for optWall
wp_remove1 = optWallsStruct.wp(idx,1:2);
wp_remove2 = optWallsStruct.wp(idx,3:4);

q_wpNew = [];

for i = 1:size(q_wp,1)
    % if waypoint is not either of the waypoints to remove, add to q_wpNew
    if ~all(q_wp(i,:) == wp_remove1) && ~all(q_wp(i,:) == wp_remove2)
        q_wpNew = [q_wpNew; q_wp(i,:)];
                
    end
end
wp_removed = [wp_remove1; wp_remove2];
    
end