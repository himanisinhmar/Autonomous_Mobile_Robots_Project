function pgon = SAPpgon(stayAwayPoints,bloatFac)

% create polygons around the stay away points
for i = 1:size(stayAwayPoints,1)
    center = [stayAwayPoints(i,1) stayAwayPoints(i,2)];
    xLeft = center(1) - bloatFac;
    xRight = center(1) + bloatFac;
    yBottom = center(2) - bloatFac;
    yUp = center(2) + bloatFac;
    vert1 = [xLeft yBottom];
    vert2 = [xLeft yUp];
    vert3 = [xRight yUp];
    vert4 = [xRight yBottom];
    pgon(i) = polyshape([vert1(1) vert2(1) vert3(1) vert4(1)],...
    [vert1(2) vert2(2) vert3(2) vert4(2)]);
end

pgon = union(pgon);

% % plot
% for i = 1:size(stayAwayPoints,1)
%    plot(stayAwayPoints(i,1),stayAwayPoints(i,2),'r.','MarkerSize',10);
%    hold on
% end
% hold on
% plot(pgon,'FaceColor','k','FaceAlpha',0.2)
% xlabel('x (m)')
% ylabel('y (m)')
% axis equal
% hold off


end

