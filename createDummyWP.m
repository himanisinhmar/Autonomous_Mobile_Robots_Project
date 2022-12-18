function optWallsStruct=createDummyWP(optWalls,distPt2Wall,bloatFactor,mapWalls,stayAwayPoints, obstacleVerts)
% Note
% 1. if inout has a nan it means that dummy wp is off the map

% 2. bloatFactor here is the bloatFactor used for creating the bloatedMap

% 3. CURRENTLY NOT CONSIDERING the case where the dp is in an obstacle right
% after it has been placed, assuming this wont happen. 

% 4. Also assuming the safeLineSeg will have some length free of stay away 
% point's bloated square

%
%     %parameter
%     depthSensorMinRange=0.2;
%     distPt2Wall > bloatfactor(aka robotRadius) + depthSensorMinRange
    
    mapWalls=[mapWalls;optWalls];
%     obstacleVerts = polygonalWorld(mapWalls,stayAwayPoints,bloatFactor);

    %finding dimensions
    NOW=size(optWalls,1); %number of walls
    NOS=size(obstacleVerts,2);
    
    pGons=[];
    %creating the polygons and unionizing them into 1 big polygon
    for j=1:size(obstacleVerts,2)
        pnew = polyshape(obstacleVerts{j}(1:end-1,1), obstacleVerts{j}(1:end-1,2));
        pGons=[pGons; pnew];
%         plot(pnew);
        hold on;
    end

    obsPoly=union(pGons);
%     plot(obsPoly)
    
    SAPPoly = SAPpgon(stayAwayPoints,bloatFactor);
    
    %finding map boundaries
    maxX=max(max(mapWalls(:,1)),max(mapWalls(:,3)));
    minX=min(min(mapWalls(:,1)),min(mapWalls(:,3)));
    
    maxY=max(max(mapWalls(:,2)),max(mapWalls(:,4)));
    minY=min(min(mapWalls(:,2)),min(mapWalls(:,4)));
      
    for i=1:NOW
        %create a line parallel to the optional wall that passes through
        %the midpoint as has a length reduced by 2*bloatfactor (to account
        %for edges
        [line1MP line2MP wallMP line1 line2]=safeLineSegCreator(optWalls(i,:),distPt2Wall,bloatFactor);
        
        %check if this line intersects with an obstacle JUST FOR SAFETY,
        %NOT DOING ANYTHING WITH THIS INFO EXCEPT DISPLAYING IT
        [inObs1 multiIntersectObs1 inOBSseg1 outOBSseg1]=checkDPIntersectPgon(line1,obsPoly);
        [inObs2 multiIntersectObs2 inOBSseg2 outOBSseg2]=checkDPIntersectPgon(line2,obsPoly); 
%             if inObs1
%                 disp('lineseg 1 of opt wall ')
%                 disp(i)
%                 disp('intersects obstacle')
%                 disp(' ')
%             end
%             
%             if inObs2
%                 disp('lineseg 2 of opt wall ')
%                 disp(i)
%                 disp('intersects obstacle')
%                 disp(' ')
%             end
                
        %check if this line intersects with a stay away point
        [inSAP1 multiIntersectSAP1 inSAPseg1 outSAPseg1]=checkDPIntersectPgon(line1,SAPPoly);
        [inSAP2 multiIntersectSAP2 inSAPseg2 outSAPseg2]=checkDPIntersectPgon(line2,SAPPoly);
        
        
            %if no, pick the midpoint of the line for stay away point
            if (inSAP1==0 && inObs1==0)
                dp1=line1MP;
            elseif (~isempty(outOBSseg1))                                   
                dp1=findSpotOnLine(outOBSseg1); 
            else
%                 disp('unable to find dummy way point 1 for opt wall')
%                 disp(i)
                   dp1=line1MP; 
            end
            
            if (inSAP2==0 && inObs2==0)
                dp2=line2MP;
            elseif (~isempty(outOBSseg2))                                   
                dp2=findSpotOnLine(outOBSseg2); 
            else
%                 disp('unable to find dummy way point 2 for opt wall')
%                 disp(i)
                dp2=line2MP;
            end
            
%             if any(isnan(dp1)
%                 disp('dummy point 1 not valid for wall')
%                 disp(i)
%             end
%             
%             if any(isnan(dp2)
%                 disp('dummy point 2 not valid for wall')
%                 disp(i)
%             end
       
            %remove pts that are outside the wall boundaries
            if ((dp1(1)>maxX || dp1(1)<minX) || (dp1(2)>maxY || dp1(2)<minY))
                dp1(1)=nan;
                dp1(2)=nan;
                inPoly1=nan;
                disp('dummy way point 1 out of bounds for pt')
                disp(i)
            end
            if ((dp2(1)>maxX || dp2(1)<minX) || (dp2(2)>maxY || dp2(2)<minY))
                dp2(1)=nan;
                dp2(2)=nan;
                inPoly2=nan;
                disp('dummy way point 2 out of bounds for pt')
                disp(i)
            end
            
            inPoly1=checkPtinPoly(dp1,obsPoly);
            inPoly2=checkPtinPoly(dp2,obsPoly);
            
            %color array
            colorArray=[0.8500, 0.3250, 0.0980;...
            0.9290, 0.6940, 0.1250;...
            0.4940, 0.1840, 0.5560;...
            0.4660, 0.6740, 0.1880;...
            0.6350, 0.0780, 0.1840;] ;
            
%             plotting for testing
%             if i==1
%                 plot([optWalls(i,1) optWalls(i,3)],[optWalls(i,2) optWalls(i,4)],'--','color',colorArray(mod(i,5)+1,:),'DisplayName','Optional Wall')
%                 hold on
%                 plot(dp1(1),dp1(2),'*','color',colorArray(mod(i,5)+1,:),'DisplayName','Dummy Way Point')
%                 hold on
%                 plot(dp2(1),dp2(2),'*','color',colorArray(mod(i,5)+1,:),'HandleVisibility','off')
%                 hold on
%             else
%                 plot([optWalls(i,1) optWalls(i,3)],[optWalls(i,2) optWalls(i,4)],'--','color',colorArray(mod(i,5)+1,:),'HandleVisibility','off')
%                 hold on
%                 plot(dp1(1),dp1(2),'*','color',colorArray(mod(i,5)+1,:),'HandleVisibility','off')
%                 hold on
%                 plot(dp2(1),dp2(2),'*','color',colorArray(mod(i,5)+1,:),'HandleVisibility','off')
%                 hold on
%             end
            
            optWallsStruct.walls(i,:)=[i optWalls(i,:)];
            optWallsStruct.wp(i,:)=[dp1(1) dp1(2) dp2(1) dp2(2)];
            optWallsStruct.inout(i,:)=[inPoly1 inPoly2];
%             
        
        
    end
   
end