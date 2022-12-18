% color's rgb values
purple = [0.4940, 0.1840, 0.5560];
purpleL = purple  +0.4;
darkG = [0, 0.5, 0];
crimson = [0.6350, 0.0780, 0.1840];
darkY = [0.9290, 0.6940, 0.1250];
teal = [0 0.75 0.75];
global dataStore;
load('compMap.mat');
originalMap = map;
map = dataStore.map;
obstacleVerts = polygonalWorld(map,stayAwayPoints,0.3);
% output  dataStore.wp
visitedWaypoints = dataStore.wp;
wp = [waypoints;ECwaypoints];
optWallsNotChecked = [];
res1 = 1;
res2 = 1;
for i = 1:size(optWalls,1)
    if ~isempty(dataStore.optWallsYes)
        [res1,~] = ismember(optWalls(i,:),dataStore.optWallsYes,'rows');
    end
    if ~isempty(dataStore.optWallsNo)
        [res2,~] = ismember(optWalls(i,:),dataStore.optWallsNo,'rows');
    end
    if ~res1 || ~res2
        optWallsNotChecked = [optWallsNotChecked;optWalls(i,:)];
    end
    res1 = 1;
    res2 = 1;
end
figure;
% for i = 1:length(obstacleVerts)
%     pnew = polyshape(obstacleVerts{i}(1:end-1,1), obstacleVerts{i}(1:end-1,2));
%     obst = plot(pnew,'FaceColor','k','FaceAlpha',0.05);
%     hold on
% end
% hold on
% plot the current map in black
map = [originalMap;dataStore.optWallsYes];
for i = 1:size(map,1)
    w = plot([map(i,1),map(i,3)],[map(i,2),map(i,4)],'k','LineWidth',2);
    hold on
end
hold on
% plot the optional walls that hasn't been checked in red
if ~isempty(optWallsNotChecked)
for i = 1:size(optWallsNotChecked,1)
   notChecked = plot([optWallsNotChecked(i,1),optWallsNotChecked(i,3)],...
                [optWallsNotChecked(i,2),optWallsNotChecked(i,4)]...
                ,'r','LineWidth',2); 
   hold on
end
end
hold on
% plot the trajectory
traj = plot(dataStore.pose(:,1),dataStore.pose(:,2),'b.');
for i = 1:size(visitedWaypoints,1)
    visitedWP = plot(dataStore.wp(i,1),dataStore.wp(i,2),'mo',...
        'MarkerSize',15,'LineWidth',2);
    hold on
end
for i = 1:size(wp,1)
   wayP = plot(wp(i,1),wp(i,2),'.','color',purple,'MarkerSize',25); 
end
if ~isempty(optWallsNotChecked)
    legend([notChecked w traj visitedWP,wayP],'optWalls not checked',...
        'updated map','trajectory','visited waypoints','waypoints');
else
    legend([w traj,visitedWP,wayP],'updated map','trajectory',...
        'visited waypoints','waypoints');
end
xlabel('x (m)')
ylabel('y (m)')
axis equal
title('Final plot')

