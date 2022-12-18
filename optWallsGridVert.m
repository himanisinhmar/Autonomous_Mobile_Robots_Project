function [vertsOptWalls,pgon] = optWallsGridVert(optWalls,bloatFacAlong,bloatFacNormal)

originalOptWalls = optWalls;

for i = 1:size(optWalls,1)
    % unit vec along optional wall
    x1 = optWalls(i,1);x2 = optWalls(i,3);y1 = optWalls(i,2);y2 = optWalls(i,4);
    unitVec = [(x1-x2) (y1-y2)];
    unitVec = unitVec/norm(unitVec);
    % shrink the wall at both end points by bolatfac
    optWalls(i,3:4) = optWalls(i,3:4) + bloatFacAlong*unitVec;   
    optWalls(i,1:2) = optWalls(i,1:2) - bloatFacAlong*unitVec;
    % find the 4 vertices at end points of the walls
    normVec = [unitVec(2) -unitVec(1)]; % normal vector to the line
    vert1 = optWalls(i,1:2) + normVec*bloatFacNormal;
    vert2 = optWalls(i,3:4) + normVec*bloatFacNormal;
    vert3 = optWalls(i,3:4) - normVec*bloatFacNormal;
    vert4 = optWalls(i,1:2) - normVec*bloatFacNormal;    
    vertsOptWalls{i} = [vert1;vert2;vert3;vert4];
    
end 

% given the vertices for each bloatpoly create polygons
% using polyshape
for i = 1:length(vertsOptWalls)
    vert1 = vertsOptWalls{i}(1,:);
    vert2 = vertsOptWalls{i}(2,:);
    vert3 = vertsOptWalls{i}(3,:);
    vert4 = vertsOptWalls{i}(4,:);
    pgon(i) = polyshape([vert1(1) vert2(1) vert3(1) vert4(1)],...
        [vert1(2) vert2(2) vert3(2) vert4(2)]);
    
end

% plot 
% for i = 1:size(originalOptWalls,1)
%     w = plot([originalOptWalls(i,1),originalOptWalls(i,3)],...
%         [originalOptWalls(i,2),originalOptWalls(i,4)],'k','LineWidth',2);
%     hold on
% end
% plot(pgon,'FaceColor','r','FaceAlpha',0.2)
% xlabel('x (m)')
% ylabel('y (m)')
% axis equal
% hold off

end

