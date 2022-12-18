function ECWPsnearOptWall=checkECWPnearOptWall(ECwaypoints,optWallsStruct,minLenECWPToOptWall)
%     OUPUT
%     ECWPsnearOptWall  - n-by-3 [wall id from optWallsStruct, ECWPxcoord, ECWPycoord]
%     note: there can be multiple wall ids each corresponding to a unique ECWP
%     note: there can ALSO be multiple ECWP each corresponding to a unique wall id
    

    % assumptions: line segment connects wp to only midpoint of the opt wall    
    
    %first checking if the distance from the ECWP to the midpoint of the
    %wall is within minLenECWPTtoOptWall
    optWalls=optWallsStruct.walls(:,2:5);
    optWallsMidPoints=[(optWalls(:,1)+optWalls(:,3))/2 (optWalls(:,2)+optWalls(:,4))/2];
    D=pdist2(optWallsMidPoints,ECwaypoints); %row i correspond to distances of ECWP from wall i
    
    NOW=size(optWalls,1);
    NOECWP=size(ECwaypoints,1);
    
    idx=find(D <= minLenECWPToOptWall);
    
    [optWallNum,ECWPnum]=ind2sub([NOW NOECWP],idx);
    
    ECWPsnearOptWall=[optWallNum ECwaypoints(ECWPnum,:)];

end

    


