function [flattenedParticles,totalNOP]=pfInitializer (waypoints,squareWidth,totalNumOfParticles,secondInitializer,thirdInitializer,pmTheta,currentTheta)
    
    %number of particles per WP
    NOWP=size(waypoints,1); particlesPerWP=floor(totalNumOfParticles/NOWP);
    
    %breaking down into x y 
    xPoints=waypoints(:,1); yPoints=waypoints(:,2);
    
    %creating offset Tile
%     offsetXY=((rand(1,particlesPerWP))*squareWidth)-squareWidth/2; offsetTileXY=repmat(offsetXY,NOWP,1);    
    offsetX=((rand(1,particlesPerWP))*squareWidth)-squareWidth/2; offsetTileX=repmat(offsetX,NOWP,1);    
    offsetY=((rand(1,particlesPerWP))*squareWidth)-squareWidth/2; offsetTileY=repmat(offsetY,NOWP,1); 
    
    %creating xArr and yArr
%     xArr=repmat(xPoints,1,size(offsetTileXY,2))+offsetTileXY; yArr=repmat(yPoints,1,size(offsetTileXY,2))+offsetTileXY;
    xArr=repmat(xPoints,1,size(offsetTileX,2))+offsetTileX; yArr=repmat(yPoints,1,size(offsetTileY,2))+offsetTileY;

    %adding thetaArr and arranging the columns in x y theta form
%     thetaArr=(rand(NOWP,particlesPerWP)*(2*pi()))-pi();
    if secondInitializer
        thetaArr  = -deg2rad(40) + rand(NOWP,particlesPerWP)*(deg2rad(40) - (-deg2rad(40)))+currentTheta;
    elseif thirdInitializer
        thetaArr  = -deg2rad(pmTheta) + rand(NOWP,particlesPerWP)*(deg2rad(pmTheta) - (-deg2rad(pmTheta)))+currentTheta;
    else
        thetaArr  = rand(NOWP,particlesPerWP)*(2*pi );
    end

    joinedArr=[xArr yArr thetaArr]; 
    joinedArr=reshape(permute(reshape(joinedArr,size(joinedArr,1),[],3),[1 3 2]),size(joinedArr,1),[]);
    
    %flattening it so it comes to the form [x1 y1 theta1...xNOWP yNOWP thetaNOWP]
    joinedArr=joinedArr';joinedArr=joinedArr(:);flattenedParticles=joinedArr';
    
    %total number of particles after rounding down
    totalNOP=particlesPerWP*NOWP;

end
    