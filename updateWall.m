function [wall, probList] = updateWall(occ_grid, optWallsStruct, optWalls,optWallsNotChecked,probList,threshold)
% INPUT
%       occ_grid    occupancy grid probabilities calculated from logOddsDepth
%       optWallsStruct      struct from createDummyWP 
%                           optWallsStruct.walls     [ID x1 y1 x2 y2]
%       probList    probabilities of occupancy for each wall     1x(# of optWalls)
% 
% OUTPUT
%       wall: n x 6 array, n being # of walls updated, [ID x1 y1 x2 y2 occ] 
%             if no walls are updated, the wall output is an empty list

if any(occ_grid~= 0.5)       % if a wall has been updated
    wallIdx = find(occ_grid~= 0.5);    % all the walls that have been updated
    

    wall = zeros(length(wallIdx),6);
    for j = 1:length(wallIdx)       % find ID and coords of each wall
        NotcheckedBefore =  find(ismember(optWallsNotChecked(:,2:end),optWalls(wallIdx(j),:), 'rows'),1);
        if ~isempty(NotcheckedBefore)      % if wall has not been checked before
            % find ID
            id = find(ismember(optWallsStruct.walls(:,2:end),optWalls(wallIdx(j),:), 'rows'),1);

    %         id = find(ismember(optWallsStruct.walls, optWalls(wallIdx,:), 'rows'),1);
            coords = optWalls(wallIdx(j),:); % [x1 y1 x2 y2]
            
            if occ_grid(wallIdx(j)) < 0.5     %unoccupied
                probList(wallIdx(j)) = probList(wallIdx(j)) - threshold;
            else                     % occupied
                probList(wallIdx(j)) = probList(wallIdx(j)) + threshold;
            end
            
            if probList(wallIdx(j)) <= 0.2        % unoccupied
                occ = 0;
            elseif probList(wallIdx(j)) >= 0.8    % occupied
                occ = 1;
            else                            % not fully sure yet
                occ = 0.5; 
            end
            wall(j,:) = [id coords occ];
        else
            wall(j,:) = repmat(NaN, 1, 6);
        end

    end
%     wall = rmmissing(wall);
    wall(find(isnan(wall(:,1))),:) = [];

else
    wall = [];
end


end