function obstacleVerts = buildWalls(map, delta)
 % given map: a matrix, and a desired thickness, construcut wall obstacles
 % in ccw direction
    half = delta/2;
    obstacleVerts{1, size(map,1)} = [];
    
    x1 = map(:,1); y1 = map(:,2); x2 = map(:,3); y2 = map(:,4);
    for i = 1:size(map,1)
        if y1(i) == y2(i)      % horizontal line, starting from left point
            if x2(i) > x1(i)   % if second endpoint is on the left
             p1 = [x2(i), y2(i)-half];
             p2 = [x2(i), y2(i)+half];
             p3 = [x1(i), y1(i)+half];
             p4 = [x1(i), y1(i)-half];
            elseif x1(i) > x2(i)   % if first endpoint is on the left
             p1 = [x1(i), y1(i)-half];
             p2 = [x1(i), y1(i)+half];
             p3 = [x2(i), y2(i)+half];
             p4 = [x2(i), y2(i)-half];
            end

        elseif x1(i) == x2(i)      % vertical line, starting from top point
            if y2(i) > y1(i) % if second endpoint is on top
             p1 = [x2(i)+half, y2(i)];
             p2 = [x2(i)-half, y2(i)];
             p3 = [x1(i)-half, y1(i)];
             p4 = [x1(i)+half, y1(i)];
            elseif y1(i) > y2(i) % if first endpoint is on top
             p1 = [x1(i)+half, y1(i)];
             p2 = [x1(i)-half, y1(i)];
             p3 = [x2(i)-half, y2(i)];
             p4 = [x2(i)+half, y2(i)];
            end
        end
        
        obstacleVerts{i} = [p1; p2; p3; p4];

    end

end