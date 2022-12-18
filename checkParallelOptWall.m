function worthIt=checkParallelOptWall(Path,optWallsStruct,angDeviation,start,lastSegLenThresh,thetaToTurnThresh)
%     INPUT
%     angDeviation is in degrees

%     OUTPUT
%     worthIt=1  means anglebetween current path and corresponding optWall(s) is > angDeviation
%               means go to the dummy way point
    
    %assuming:
    %1. the dummy way point is at the end of the path
    %2. dummy way points of different walls do not overlap
    %3. the order of the optional walls (row wise) stays consistent since
    %creatdummyWP field wise
    
    dWP=Path(end,:);
    %creating the first line segment that the robot is currently following
    if size(Path,1)>1 
        p1=Path(end-1,:);
        p2=dWP;
    else %when dummy way point is in line of sight
        p1=start;
        p2=dWP;
    end
    
    lastSegLen=norm(p1-p2);
    
    if size(Path,1)>2 && ~isempty(Path)
        q1=Path(end-2,:);
        thetaToTurn = 180 - angleFinder(q1,p1,p2);
        if (lastSegLen<lastSegLenThresh && thetaToTurn>thetaToTurnThresh)
            worthIt=0;
            return
        end
    elseif size(Path,1)==2
        q1=start;
        thetaToTurn = 180 - angleFinder(q1,p1,p2);
        if (lastSegLen<lastSegLenThresh && thetaToTurn>thetaToTurnThresh)
            worthIt=0;
            return
        end
    else %if size of the path is 1
        if (lastSegLen<lastSegLenThresh)
            worthIt=0;
            return
        end    
    end
    
    %find the wall coordinates (could be more than one wall) corresponding to the dummywaypoint
    set1=optWallsStruct.wp(:,1:2);
    set2=optWallsStruct.wp(:,3:4);
    
    %extracting indices which match the dWP
    interSet1=repmat(dWP,size(set1,1),1)-set1;
    interSet1(abs(interSet1)<0.001)=0;
    indSet1=~any(interSet1,2);
    interSet2=repmat(dWP,size(set2,1),1)-set2;
    interSet2(abs(interSet2)<0.001)=0;
    indSet2=~any(interSet2,2);
      
    %creating the combined set from both sets (both dummy points) of walls to be checked
    qSet1=[optWallsStruct.walls(indSet1,2:3) optWallsStruct.walls(indSet1,4:5)];
    qSet2=[optWallsStruct.walls(indSet2,2:3) optWallsStruct.walls(indSet2,4:5)];
    qSet=[qSet1;qSet2];
    
    worthIt=1;
    
    for i=1:size(qSet,1)
        q1=[qSet(i,1) qSet(i,2)];
        q2=[qSet(i,3) qSet(i,4)];
        
        p3=(q2-q1)+p2;
        theta=angleFinder(p1,p2,p3);
        if theta<=90 %if theta is between 0 and 90
           if (theta<angDeviation)
               worthIt=0;
               return
           end
        else %if theta is between 90 and 180
            if (theta>(180-angDeviation))
                worthIt=0;
                return
            end
        end
    end
    
end
