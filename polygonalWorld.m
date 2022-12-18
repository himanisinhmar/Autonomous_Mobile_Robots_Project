function obstacleVerts = polygonalWorld(map,stayAwayPoints,bloatFactor)

% creates polygons for obstacles and walls
% INPUTS:
%           map                    with optional walls
%           StayAwayPoints         locations
%           bloatFactor 		   by how much must the walls be bloated
% 
% OUTPUTS:
%           obstacleverts          vertices of all polygons and vertices of triangles 
%                                  around stay-away points

% AMR Final Competition
% HIMANI SINHMAR

originalMap = map;
minX = min(min(map(:,1)),min(map(:,3)));
maxX = max(max(map(:,1)),max(map(:,3)));
minY = min(min(map(:,2)),min(map(:,4)));
maxY = max(max(map(:,2)),max(map(:,4)));

% construct polygons from given map
bloatFac = bloatFactor;
for i = 1:size(map,1)
    modifiedMap = originalMap;
    % extend the wall at both end points
    x1 = map(i,1);x2 = map(i,3);y1 = map(i,2);y2 = map(i,4);
    unitVec = [(x1-x2) (y1-y2)];
    unitVec = unitVec/norm(unitVec);
    modifiedMap(i,:) = [];
    [res1,~] = ismember([x1 y1],modifiedMap(:,1:2),'rows');
    [res2,~] = ismember([x1 y1],modifiedMap(:,3:4),'rows');
    [res3,~] = ismember([x2 y2],modifiedMap(:,1:2),'rows');
    [res4,~] = ismember([x2 y2],modifiedMap(:,3:4),'rows');
    resA = res1 || res2;
    resB = res3 || res4;
    onBoundary1 =  x1==minX || x1==maxX || y1==minY || y1==maxY;
    onBoundary2 =  x2==minX || x2==maxX || y2==minY || y2==maxY;
    if (x1==x2) || (y1==y2)
        map(i,3:4) = map(i,3:4) - bloatFac*unitVec;   
        map(i,1:2) = map(i,1:2) + bloatFac*unitVec;     
    else
        if ~resA && ~onBoundary1   
            map(i,1:2) = map(i,1:2) + bloatFac*unitVec;
        end
        if ~resB && ~onBoundary2 
            map(i,3:4) = map(i,3:4) - bloatFac*unitVec;
        end
    end
    % find the 4 vertices at end points of the walls
    normVec = [unitVec(2) -unitVec(1)]; % normal vector to the line
    vert1 = map(i,1:2) + normVec*bloatFac;
    vert2 = map(i,3:4) + normVec*bloatFac;
    vert3 = map(i,3:4) - normVec*bloatFac;
    vert4 = map(i,1:2) - normVec*bloatFac;    
    bloatPoly{i} = [vert1;vert2;vert3;vert4];
    
end 

% given the vertices for each bloatpoly create polygons
% using polyshape
for i = 1:length(bloatPoly)
    vert1 = bloatPoly{i}(1,:);
    vert2 = bloatPoly{i}(2,:);
    vert3 = bloatPoly{i}(3,:);
    vert4 = bloatPoly{i}(4,:);
    pgon(i) = polyshape([vert1(1) vert2(1) vert3(1) vert4(1)],...
        [vert1(2) vert2(2) vert3(2) vert4(2)]);
    
end

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
    pgon = [pgon polyshape([vert1(1) vert2(1) vert3(1) vert4(1)],...
    [vert1(2) vert2(2) vert3(2) vert4(2)])];
end

% unionize all the polygons, this take care of
% of the intersecting polygons
pgonUnion = union(pgon);
region = regions(pgonUnion);

% extract vertices of the unionized polygon
for i = 1:length(region)
    verts = region(i).Vertices;
    % remove polygons which are inside any polygon
    % except for the outer boundary
    if ~any(verts(:,1) > maxX)
        for j = 1:size(verts,1)
           if isnan(verts(j,1))               
                verts(j:end,:) = [];
                break;
           end
        end
    else % for the outer boundary
        counter = 1;
        for j = 1:size(verts)
           if isnan(verts(j,1))
                if counter ~= 1
                    verts(j:end,:) = [];
                    break;
                end
                counter = counter+1;
           end
        end
    end
    obstacleVerts{i} = [verts;verts(1,:)];
end

end

