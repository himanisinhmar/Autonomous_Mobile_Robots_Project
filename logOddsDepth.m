function Lt = logOddsDepth(xt, zt, L_prev, L0, vertsOptWalls, vertsRegWalls, sensorO)
% UPDATED FOR FINAL COMPETITION: each optWall is a cell (and nowhere else)
% 
% calculate the log-odds of cells in the grid using only depth
%              sensor measurements.
%
%   INPUTS
%       xt              pose of robot               [1 x 3]
%       zt              bump sensor measurements    [1 x 7]
%                       [BumpRight BumpLeft DropRight DropLeft DropCaster BumpFront]
% 
%       L_prev          previous log-odds grid      [(# of cells in x) x ( # of cells in y)]
%       L0              initial log-odds value       
%       
%       vertsOptWalls      polygons for each optWall  (vertices are cw)
%                            n cells, each cell has 4 x 2
%       vertsRegWalls      polygons for each regular wall  (vertices are cw)
%                            n cells, each cell has 4 x 2
% 
%       sensorO         sensor position relative to center of robot [m]     [1 x 2]
%
%   OUTPUTS
%       Lt              next log-odds grid          1 x (# of walls)

% range of sensor
minDepth = 0.175;
% maxDepth = sqrt(5.5^2+16);
maxDepth = 10;
angles = linspace(deg2rad(27), deg2rad(-27), 9);
% [m,n] = size(L_prev);
obst_pos = zeros(size(L_prev));

Lt = L_prev - L0*ones(size(L_prev));

% sensor position
sensorPose = [robot2global(xt, sensorO'), xt(3)];

ignoreMeas = 0;
for i = 1:size(zt,2) 
    
    if((zt(i) > minDepth) && (zt(i) < maxDepth) )    % within range of sensor
        
        % find endpoint of depth ray      
        x = zt(i);
        y = zt(i)*tan(angles(i));
        depth_pos = robot2global(sensorPose, [x y]);
        
%         plot([sensorPose(1),depth_pos(1)],[sensorPose(2),depth_pos(2)],"--");

        % if depth sensor sees thru walls, don't update
        lineseg = [sensorPose(1) sensorPose(2); depth_pos(1) depth_pos(2)];
        for j = 1: length(vertsRegWalls)  
            [checkReg,~] = intersect(polyshape(vertsRegWalls{j}), lineseg);
            if ~isempty(checkReg)         % if seeing thru an actual wall
                ignoreMeas = 1;
                break
            end
            
        end

        if ~ignoreMeas              % if depth ray does not see thru walls
            occWallidx = [];
            lineseg = [sensorPose(1),sensorPose(2); depth_pos(1),depth_pos(2)];
           
            % find out on what cell depthPose is in
            for j = 1: length(vertsOptWalls)    % for each wall to check
                checkOpt = intersect(polyshape(vertsOptWalls{j}), lineseg);     % if depth ray intersects optional wall
                if checkOpt            % found intersection with optWall cell
                    occWallidx = [occWallidx j];    % store occupied cell index 
%                     break
                end
            end
            
            if ~isempty(occWallidx)    % if found intersection
                for w = 1:length(occWallidx)    % for each opt wall the ray intersects
                    idx = occWallidx(w);
                    % check if endpoint of depth ray is in the wall
                    inWall = inpolygon(depth_pos(1),depth_pos(2), vertsOptWalls{idx}(:,1),vertsOptWalls{idx}(:,2));

                    if inWall       % if depth endpoint is in wall, wall is occupied
                         Lt(idx) = Lt(idx) + log(1e10);
                                                                                                             % red w/ transparency
                         depthRay1 = plot([sensorPose(1),depth_pos(1)],[sensorPose(2),depth_pos(2)],'--','color',[1 0 0 0.2]);

                    else            % optional wall cell is unoccupied
                         Lt(idx) = Lt(idx)+ log(1e-10);
                                                                                                              % green w/ transparency
                         depthRay2 = plot([sensorPose(1),depth_pos(1)],[sensorPose(2),depth_pos(2)],'--','color',[0 0.75 0 0.2]);
                    end
                end
            end


        end

    end
    ignoreMeas = 0;
end
end