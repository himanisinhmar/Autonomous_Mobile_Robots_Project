function [inBool multiIntersect in out]=checkDPIntersectPgon(line,pGon)
    %this function will tell if a line intersects more than one free region
    %if doubleIntersect is true then the line goes through Cfree Cobs then
    %Cfree

%     NOS=size(pGons,1);
%     NOS=size(obstacleVerts,2);
%     pGons=[];
%     inEdges=[];
%     qEdges=[];
%     for j=1:size(obstacleVerts,2)
%         pnew = polyshape(obstacleVerts{j}(1:end-1,1), obstacleVerts{j}(1:end-1,2));
%         pGons=[pGons; pnew];
%     end
    multiIntersect=0;
    lineSeg=[line(1) line(2); line(3) line(4)];
%     inSegs={};
%     outSegs={};
    inBool=0;
    
    %finding which polygon the optional wall is in
%     kPol=lineWhichPol(pGons,optWall);
    
    %considering the polygon that has the optional wall
 
%         pgon=pGons(kPol);
%                     if ((j==17)&(k==1))
%                         disp(j)
%                     end
        [in,out]=intersect(pGon,lineSeg);

%         if(size(in,1)==2)    
%             if (in==lineSeg) %this means the entire line segment is inside the bloating zone
%                 inBool=1;
%             else
%                 inBool=0;
%             end
%         end
%         inEdges=[inEdges; inBool];
        if ~isempty(in)
            inBool=1;
            if(sum(isnan(in),'all'))
                multiIntersect=1;
            end
        else
            inBool=0;
        end
        
   
    
    
    
    
    
%     %going through all polygons
%     for k=1:NOS
%         pgon=pGons(k);
% %                     if ((j==17)&(k==1))
% %                         disp(j)
% %                     end
%         [in,out]=intersect(pgon,lineSeg);
%         inSegs{k,1}=in;
%         outSegs{k,1}=out;
%         
%         if (isempty(outSegs)) %this means the entire line segment is inside the bloating zone
%             inBool=1;
%         else
%             inBool=0;
%         end
%         inEdges=[inEdges; inBool];
%         if ~isempty(in)
%             if(sum(isnan(in),'all'))
%                 doubleIntersect=1;
%             end
%         end
%         
%     end
%     if (sum(inEdges,'all')==0)
%         qEdges=[qEdges; lineSeg(:)'];
% %                     plot([edgesList(end,1) edgesList(end,2)],[edgesList(end,3) edgesList(end,4)])
% %                     hold on
%     end

%     if isempty(qEdges)
%         intersectBool=1; %line does intersect polygon, no obstacle free edges (qEdges)
%     else
%         intersectBool=0; %line does not intersect polygon, there exist obstacle free edges
%     end
    
    

%     doubleIntersect=~doubleIntersect;

end
