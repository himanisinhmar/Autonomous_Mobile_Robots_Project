function [check]= LineinCells(x1,y1,x2,y2, coord1, coord2, coord3, coord4)
% find cells that the line with endpoints (x1,y1),(x2,y2) occupy
% INPUTS 
%   (x1,y1)     Start position
%   (x2,y2)     End position
%   
%   coord1      vertices of cell (in cw order)
%   coord2      
%   coord3
%   coord4
% OUTPUTS
%       check     check if given cellx, celly is occupied

    check = 0;

    % if cell is within the 1/4, 3/4 block of the cell
%     if intersectPoint(x1,y1,x2,y2, cellx+cell_width/4, celly+cell_height/4, cellx+3*cell_width/4, celly+cell_height/4) ...
%                             || intersectPoint(x1,y1,x2,y2, cellx+cell_width/4, celly+3*cell_height/4, cellx+3*cell_width/4, celly+3*cell_height/4) ...
%                             || intersectPoint(x1,y1,x2,y2, cellx+cell_width/4, celly+cell_height/4, cellx+cell_width/4, celly+3*cell_height/4) ...
%                             || intersectPoint(x1,y1,x2,y2, cellx+3*cell_width/4, celly+cell_height/4, cellx+3*cell_width/4, celly+3*cell_height/4)
 
    if intersectPoint(x1,y1,x2,y2, coord1(1), coord1(2), coord2(1), coord2(2))...
    || intersectPoint(x1,y1,x2,y2, coord1(1), coord1(2), coord4(1), coord4(2))...
    || intersectPoint(x1,y1,x2,y2, coord3(1), coord3(2), coord2(1), coord2(2))...
    || intersectPoint(x1,y1,x2,y2, coord3(1), coord3(2), coord4(1), coord4(2))
        
        check =1 ;
    end



end


%     if intersectPoint(x1,y1,x2,y2, cellx, celly, cellx+cell_width, celly) ...
%                             || intersectPoint(x1,y1,x2,y2, cellx, celly, cellx, celly+cell_height) ...
%                             || intersectPoint(x1,y1,x2,y2, cellx+cell_width, celly, cellx+cell_width, celly+cell_height) ...
%                             || intersectPoint(x1,y1,x2,y2, cellx, celly+cell_height, cellx+cell_width, celly+cell_height) ...
% 


%     if intersectPoint(x1,y1,x2,y2, cellx+cell_width, celly, cellx+cell_width, celly+cell_height) ...
%                             || intersectPoint(x1,y1,x2,y2, cellx, celly+cell_height, cellx+cell_width, celly+cell_height) ...


%     if intersectPoint(x1,y1,x2,y2, cellx, celly+cell_height/2, cellx+cell_width, celly+cell_height/2) ...
%             && intersectPoint(x1,y1,x2,y2, cellx+cell_width/2, celly, cellx+cell_width/2, celly+cell_height)