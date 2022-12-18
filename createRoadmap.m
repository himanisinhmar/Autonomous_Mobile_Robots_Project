function roadmap = createRoadmap(obstacleVerts, q_wp, ccw)
% ********* UPDATED FOR FINAL COMPETITION ********* 
% given a polygonal environment, returns reduced visibility roadmap
% covering Qfree
%   INPUTS
%   
%   obstacleVerts   n-dimensional cell array of obstacle vertices 
%   q_wp            list of waypoints [x1 y1; x2 y2; ...]
%   ccw             boolean if obstacleVerts are given ccw or not
% 
%   OUTPUTS
% 
%   roadmap         roadmap struct with 
%                       roadmap.V       nx2 vertices [x y]  
%                       roadmap.E       mx4 edges [x1 y1 x2 y2]

roadmap.V = [];
roadmap.E = [];

% remove outer boundary, which is in everything above NaN in obstacleVerts{1}
obstacleVertsnoNaN = obstacleVerts;         
for i =1:size(obstacleVertsnoNaN{1},1)
    if any(isnan(obstacleVertsnoNaN{1}(i)))
        obstacleVertsnoNaN{1}(1:i,:) = [];
    end
end

% obstacleVertsnoNaN{1}(end,:) is the first row of obstacleVerts
% need to add the first element of obstacleVertsnoNaN to the end
obstacleVertsnoNaN{1}(end,:) = obstacleVertsnoNaN{1}(1,:);    


if ~ccw
    ccw = -1;
end

% find all reflexive vertices
for n = 1:length(obstacleVertsnoNaN)     % for each obstacle
    reflexV = [];

    for m = 1:size(obstacleVertsnoNaN{n},1)-1   % for each vertex of obstacle
        
        if ~isnan(obstacleVertsnoNaN{n}(m,1))
            
        
        % ~~~ find consecutive vertices ~~~
        if m == size(obstacleVertsnoNaN{n},1)-1   % if checking last vertex, need to loop back to second row
            x1 = obstacleVertsnoNaN{n}(m,1);   y1 = obstacleVertsnoNaN{n}(m,2);       % adjacent vertex
            x2 = obstacleVertsnoNaN{n}(m+1,1);   y2 = obstacleVertsnoNaN{n}(m+1,2);   % current vertex
            x3 = obstacleVertsnoNaN{n}(2,1);   y3 = obstacleVertsnoNaN{n}(2,2);       % adjacent vertex
        else  
           x1 = obstacleVertsnoNaN{n}(m,1);   y1 = obstacleVertsnoNaN{n}(m,2);          % adjacent vertex
            x2 = obstacleVertsnoNaN{n}(m+1,1);   y2 = obstacleVertsnoNaN{n}(m+1,2);     % current vertex
            x3 = obstacleVertsnoNaN{n}(m+2,1);   y3 = obstacleVertsnoNaN{n}(m+2,2);     % adjacent vertex
        end
        
        % ~~~ check if vertices are reflexive ~~~
        % 2 line segments
        l1 = [x2 y2] - [x1 y1]; l2 = [x3 y3] - [x2 y2];
        % vertices for each obstacle is given in ccw direction. so if cross
        % product of (l1, l2) is in positive khat direction, vertex is
        % reflexive
        check = ccw*dot(cross([l1 0],[l2 0]), [0 0 1]);
        if check >= 0       % vertex [x2 y2] is reflexive
            reflexV = [reflexV; x2 y2];
        end
        
        end
    end
    
    if isempty(reflexV)
        disp('here')
    end
    
    % ~~~ add edges for consecutive reflexive vertices ~~~
    for m = 1:size(obstacleVertsnoNaN{n},1)-1     % for each vertex of obstacle        
        current = [obstacleVertsnoNaN{n}(m,1) obstacleVertsnoNaN{n}(m,2)];
        next = [obstacleVertsnoNaN{n}(m+1,1) obstacleVertsnoNaN{n}(m+1,2)];
        check_curr = ismember(reflexV,current,'rows');
        check_next = ismember(reflexV,next,'rows');
        
        % if vertex is reflexive and next vertex is reflexive, add edge
        if sum(check_curr)==1 
            roadmap.V = [roadmap.V; current];
            if sum(check_next) == 1   
                roadmap.E = [roadmap.E; current next];                
            end
        end
        
    end
    
end
reflexV = roadmap.V;

% ~~~ check for bi-tangent lines ~~~
% first check if line goes inside the obstacle (is it visible?)
for i = 1:size(reflexV,1)     % for each reflexive vertex (excl. start and goal)
    for j = 1:size(reflexV,1)
        if i ~= j
            % find which obstacle vi and vj are in
            obstIdx_i = 0; obstIdx_j = 0;
            vIdx_i = []; vIdx_j = [];
            for n = 1:length(obstacleVerts)     % for each obstacle
                vIdx_i = find(ismember(obstacleVerts{n},reflexV(i,:),'rows'),1);
                
                if ~isempty(vIdx_i)
                    obstIdx_i = n;
                    break
                end
            end
            
            for n = 1:length(obstacleVerts)     % for each obstacle
                vIdx_j = find(ismember(obstacleVerts{n},reflexV(j,:),'rows'),1);
                
                if ~isempty(vIdx_j)
                    obstIdx_j = n;
                    break
                end
            end

                                
            coordi = obstacleVerts{obstIdx_i}(vIdx_i,:);
            coordj = obstacleVerts{obstIdx_j}(vIdx_j,:);
            vec_ij = coordj - coordi ;
            lineseg = [coordi; coordj];
            
            % ~~~ check if line segment intersects any obstacle ~~~
            inval = 0;
            for n = 1:length(obstacleVerts)     % for each obstacle
                polyi = polyshape(obstacleVerts{n}(1:end-1,:));
                [ini,~] = intersect(polyi,lineseg);
                if ~isempty(ini)     % if intersection happens
                    inval = 1;
                    break;
                end

%                 polyj = polyshape(obstacleVerts{obstIdx_j}(1:end-1,:));
%                 [inj,~] = intersect(polyj,lineseg); 
                
            end
            
            
%             polyi = polyshape(obstacleVerts{obstIdx_i}(1:end-1,:));
%             [ini,~] = intersect(polyi,lineseg);
%             
%             polyj = polyshape(obstacleVerts{obstIdx_j}(1:end-1,:));
%             [inj,~] = intersect(polyj,lineseg);
            
            % note: line segment directly on edge is considered inside
           
            
%             % ~~~ check if line segment +- delta intersects either obstacle ~~~
%             lineseg_new = [lineseg(1,:)-delv; lineseg(2,:)+delv];
%             if isempty(ini) && isempty(inj)
% %                 plot([lineseg_new(1,1),lineseg_new(2,1)],[lineseg_new(1,2),lineseg_new(2,2)], '--')
%             end
% 
%             
% %             polyi_new = polyshape(obstacleVerts{obstIdx_i});
%             [ini_new,~] = intersect(polyi,lineseg_new);
%             
% %             polyj_new = polyshape(obstacleVerts{obstIdx_j});
%             [inj_new,~] = intersect(polyj,lineseg_new);
%             % note: line segment directly on edge is considered inside
                        
            
            % ~~~ check if line segment +- delta intersects either obstacle ~~~
            
            % find neighboring vertices for vertex i
            vi = find(ismember(obstacleVerts{obstIdx_i},coordi,'rows'),1);
            if vi == 1
                vi1 = obstacleVerts{obstIdx_i}(size(obstacleVerts{obstIdx_i},1) - 1,:);
                vi2 = obstacleVerts{obstIdx_i}(vi+1,:);
            else
                vi1 = obstacleVerts{obstIdx_i}(vi-1,:);
                vi2 = obstacleVerts{obstIdx_i}(vi+1,:);
            end
            vec1i = obstacleVerts{obstIdx_i}(vi,:) - vi1;     % ccw vec
            vec2i = vi2 - obstacleVerts{obstIdx_i}(vi,:);
            
            % find neighboring vertices for vertex j
            vj = find(ismember(obstacleVerts{obstIdx_j},coordj,'rows'),1);
            
            if vj == 1
                vj1 = obstacleVerts{obstIdx_j}(size(obstacleVerts{obstIdx_j},1) - 1,:);
                vj2 = obstacleVerts{obstIdx_j}(vj+1,:);
            else
                vj1 = obstacleVerts{obstIdx_j}(vj-1,:);
                vj2 = obstacleVerts{obstIdx_j}(vj+1,:);
            end
            vec1j = obstacleVerts{obstIdx_j}(vj,:) - vj1;     % ccw vec
            vec2j = vj2 - obstacleVerts{obstIdx_j}(vj,:);
            
            % for each check1 and check2, the signs should be opposite
            check1i = dot(cross([vec1i 0],[vec_ij 0]), [0 0 1]);
            check2i = dot(cross([vec2i 0],[vec_ij 0]), [0 0 1]);
            check1j = dot(cross([vec1j 0],[vec_ij 0]), [0 0 1]);
            check2j = dot(cross([vec2j 0],[vec_ij 0]), [0 0 1]);

            
%             || all( abs(lineseg(1,:)-ini_new(1,:)) < 1e-11*ones(1,2) )
%             if (isempty(ini) && isempty(inj))     % line segment doesn't hit obstacle
            if ~inval     % line segment doesn't hit obstacle

                if sign(check1i)~= sign(check2i) && sign(check1j)~= sign(check2j)                    
                    E1 = [lineseg(1,:) lineseg(2,:)]; E2 = [lineseg(2,:) lineseg(1,:)];
                    [~,e1] = ismember(roadmap.E,E1,'rows');
                    [~,e2] = ismember(roadmap.E,E2,'rows');
                    if sum(e1)==0 && sum(e2)==0
                        roadmap.E = [roadmap.E; lineseg(1,:) lineseg(2,:)];
                    end
                end
            end

        end
        
    end
    
end

% add visible edges for q_wp

endpts = q_wp;
roadmap.V = [roadmap.V; endpts];

for p = 1:size(q_wp,1)     % for each waypoint
    for j = 1:size(reflexV,1)  % for each reflexive vertex
        a = 0;
        coord1 = endpts(p,:);
        coord2 = reflexV(j,:);
        qseg = [coord1; coord2];
        for n = 1:length(obstacleVerts)     % for each obstacle
            % ~~~ check if line segment intersects obstacle ~~~
            obs = polyshape(obstacleVerts{n}(1:end-1,:));
            [in,~] = intersect(obs,qseg);

            if ~isempty(in)     % line segment hits obstacle
                a = 1;
                break
            end
        end
        if ~a           % no obstacles were hit
            roadmap.E = [roadmap.E; qseg(1,:) qseg(2,:)];
        end
    end
end




end


