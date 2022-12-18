function [wp, Path] = findNextWP(roadmap, q_wp, q_start)
% given current waypoint, find the next closest (shortest shortest path) waypoint to go to next
% INPUTS:
%         roadmap 		roadmap struct with 
%                             roadmap.V       n x 2 vertices [x y]  
%                             roadmap.E       m x 4 edges [x1 y1 x2 y2]
%         q_wp 			list of waypoints [x1 y1; x2 y2; ?]
%         q_start 		current waypoint      1 x 2
% OUTPUTS:
%         wp 			next waypoint 		[x y]
%         path 			list of vertices to go to 	[x1 y1; x2 y2; ?]  (nx2)


WP = struct('wp', [], 'dist', 0, 'path', []);
WPlist = [];     % list of waypoints to assign as q_goal

closest_idx = 0;
for i = 1:size(q_wp,1)
    if ~all(q_wp(i,:) == q_start)     % exclude the current waypoint
        q_goal = q_wp(i,:);
        WP.wp = q_goal;
		[dist,Path] = findPath(roadmap,q_start,q_goal);  % function already written
        % flatten path and add to all_paths
        if ~isempty(Path)    % not a disconnected waypoint
            WP.dist = dist;
            if isempty(WPlist)      % first wp
                closest_idx = 1;
            elseif dist < WPlist(closest_idx).dist    % find min distance
                closest_idx = length(WPlist)+1;
            end
            flattenP = reshape(Path', size(Path,1)*size(Path,2),1);  % flatten path
            WP.path = flattenP';
            WPlist = [WPlist; WP];
        end
    end
end

if closest_idx <= 0 || closest_idx > size(WPlist,1)
    disp('No more waypoints to go to')
    wp = q_start;
    Path = q_start;
else
    wp = WPlist(closest_idx).wp;
    Path = WPlist(closest_idx).path;
    Path = reshape(Path', 2, size(Path,2)/2)';    % unflatten path
end

% if ~isempty(Path)
%     wp = WPlist(closest_idx).wp;
%     Path = WPlist(closest_idx).path;
%     Path = reshape(Path', 2, size(Path,2)/2)';    % unflatten path
% else
%     disp('No more waypoints to go to')
%     wp = q_start;
%     Path = q_start;
% end


end