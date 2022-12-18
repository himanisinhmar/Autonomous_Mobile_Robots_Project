function maxV=speedControl(currentPose4Vel,gotopt,Path,start,angThres,radiusAccWP,maxVUpper,maxVLower,maxVSlow,smallSegLenThresh,normSegLenThresh,highVelSegLenThresh,cmdV)
    
    %default value that will get overwritten
    thetaNext=70;

    if isempty(Path)
        maxV = maxVLower;
        return
    end
    
    if gotopt~=1 
        dis = norm([(Path(gotopt-1,1)-Path(gotopt,1));(Path(gotopt-1,2)-Path(gotopt,2))],2);
        devBack=norm(currentPose4Vel-Path(gotopt-1,:));
        devFor=norm(currentPose4Vel-Path(gotopt,:));
    else
        dis = norm([(start(1)-Path(gotopt,1));(start(2)-Path(gotopt,2))],2);
        devBack=norm(currentPose4Vel-start);
        devFor=norm(currentPose4Vel-Path(gotopt,:));
    end

    %calculating next segment distance (for hard braking)
    if size(Path,1)>1
        if gotopt==size(Path,1) %if you're on the last segment
            nextSegDis=-Inf;
            hardBrake=0; %FOR NOW I'M NOT BRAKING AT THE LAST SEGMENT
%         elseif gotopt==1 %if you're on the first segment
%             nextSegDis=norm(start-Path(gotopt,:));
%                 if nextSegDis<smallSegLenThresh
%                     hardBrake=1;
%                 else
%                     hardBrake=0;
%                 end
        else %for any other segment including first segment
            nextSegDis=norm(Path(gotopt+1,:)-Path(gotopt,:));
                if nextSegDis<smallSegLenThresh
                    hardBrake=1;
                else
                    hardBrake=0;
                end
        end
    else %line of sight case where there is only one way point. SAME AS BEING ON LAST SEGMENT FOR NOW
        hardBrake=0;
    end
    
    
    if size(Path,1)>2
        if gotopt==1 %for first segment %cant mess with this because if start at opposite orientation max speed can mess us up play safe
            thetaNext=angleFinder(start,Path(gotopt,:),Path(gotopt+1,:));
            if (thetaNext<angThres) %accelerating in END of seg if next theta is small
                if (devBack>radiusAccWP&&dis>highVelSegLenThresh&&cmdV>0)
                    maxV=maxVUpper;
                else
                    if (devFor>radiusAccWP&& dis>normSegLenThresh &&devBack>radiusAccWP&&cmdV>0)
                        maxV=maxVUpper;
                    else
                        maxV=maxVLower;
                    end
                end
            else
                if (devFor>radiusAccWP&& dis>normSegLenThresh &&devBack>radiusAccWP&&cmdV>0)
                    maxV=maxVUpper;
                else
                    maxV=maxVLower;
                end
            end
        elseif gotopt==size(Path,1) %for final segment thetaNext is the final theta play safe
            thetaPrev=angleFinder(Path(gotopt-2,:),Path(gotopt-1,:),Path(gotopt,:));
            if (thetaPrev<angThres) %accelerating in START of seg if prev theta is small
                if (devFor>radiusAccWP&&dis>highVelSegLenThresh&&cmdV>0)
                    maxV=maxVUpper;
                else
                    if (devFor>radiusAccWP&& dis>normSegLenThresh &&devBack>radiusAccWP&&cmdV>0)
                        maxV=maxVUpper;
                    else
                        maxV=maxVLower;
                    end
                end
            else
                if (devFor>radiusAccWP&& dis>normSegLenThresh &&devBack>radiusAccWP&&cmdV>0)
                    maxV=maxVUpper;
                else
                    maxV=maxVLower;
                end
            end
        elseif gotopt==2 %gotopt is 2
            thetaNext=angleFinder(Path(gotopt-1,:),Path(gotopt,:),Path(gotopt+1,:));
            thetaPrev=angleFinder(start,Path(gotopt-1,:),Path(gotopt,:));
            if (thetaNext<angThres) %case where we accelerate in the END OF THE SEG
                if (thetaPrev<angThres&&dis>highVelSegLenThresh&&cmdV>0) %if both angles are small then ACCELERATE
                    maxV=maxVUpper;
                else
                    if (devBack>radiusAccWP&&dis>highVelSegLenThresh&&cmdV>0) %if we are far away enough from the START WP of seg then ACCELERATE
                        maxV=maxVUpper;
                    else
                        if (devFor>radiusAccWP&& dis>normSegLenThresh &&devBack>radiusAccWP&&cmdV>0)
                            maxV=maxVUpper;
                        else
                            maxV=maxVLower;
                        end
                    end
                end
            elseif (thetaPrev<angThres) %case where we accelerate in the START OF THE SEG
                if (thetaNext<angThres&&dis>highVelSegLenThresh&&cmdV>0) %if both angles are small then ACCELERATE
                    maxV=maxVUpper;
                else
                    if (devFor>radiusAccWP&&dis>highVelSegLenThresh&&cmdV>0) %if we are far away enough from the END WP of seg then ACCELERATE
                        maxV=maxVUpper;
                    else
                        if (devFor>radiusAccWP&& dis>normSegLenThresh &&devBack>radiusAccWP&&cmdV>0)
                            maxV=maxVUpper;
                        else
                            maxV=maxVLower;
                        end
                    end
                end
            else
                if (devFor>radiusAccWP && dis>normSegLenThresh &&devBack>radiusAccWP&&cmdV>0)
                    maxV=maxVUpper;
                else
                    maxV=maxVLower;
                end
            end
        else %any other number in between
            thetaNext=angleFinder(Path(gotopt-1,:),Path(gotopt,:),Path(gotopt+1,:));
            thetaPrev=angleFinder(Path(gotopt-2,:),Path(gotopt-1,:),Path(gotopt,:));
            if (thetaNext<angThres) %case where we accelerate in the END OF THE SEG
                if (thetaPrev<angThres&&dis>highVelSegLenThresh&&cmdV>0) %if both angles are small then ACCELERATE
                    maxV=maxVUpper;
                else
                    if (devBack>radiusAccWP&&dis>highVelSegLenThresh&&cmdV>0) %if we are far away enough from the START WP of seg then ACCELERATE
                        maxV=maxVUpper;
                    else
                        if (devFor>radiusAccWP && dis>normSegLenThresh &&devBack>radiusAccWP&&cmdV>0)
                            maxV=maxVUpper;
                        else
                            maxV=maxVLower;
                        end
                    end
                end
            elseif (thetaPrev<angThres) %case where we accelerate in the START OF THE SEG
                if (thetaNext<angThres&&dis>highVelSegLenThresh&&cmdV>0) %if both angles are small then ACCELERATE
                    maxV=maxVUpper;
                else
                    if (devFor>radiusAccWP&&dis>highVelSegLenThresh&&cmdV>0) %if we are far away enough from the END WP of seg then ACCELERATE
                        maxV=maxVUpper;
                    else
                        if (devFor>radiusAccWP&& dis>normSegLenThresh &&devBack>radiusAccWP&&cmdV>0)
                            maxV=maxVUpper;
                        else
                            maxV=maxVLower;
                        end
                    end
                end
            else
                if (devFor>radiusAccWP && dis>normSegLenThresh &&devBack>radiusAccWP&&cmdV>0)
                    maxV=maxVUpper;
                else
                    maxV=maxVLower;
                end
            end
        end

%             gotopt
%  
%             dis
% 
%             maxV
% 
%             thetaNext
% 
%             thetaPrev
        
    elseif size(Path,1)==2 
        if gotopt==1
%                     thetaNext=angleFinder(start,Path(gotopt,:),Path(gotopt+1,:));
%                     if (thetaNext<angThres)
%                         maxV = maxVUpper;
%                         thetaNext=angThres+5;
%                     else
                if (devFor>radiusAccWP && dis>normSegLenThresh &&devBack>radiusAccWP&&cmdV>0)
                    maxV=maxVUpper;
                else
                    maxV=maxVLower;
                end
%                         thetaNext=angThres+5;
%                     end
        else %for last segment we play safe
            if (devFor>radiusAccWP && dis>normSegLenThresh &&devBack>radiusAccWP&&cmdV>0)
                maxV=maxVUpper;
            else
                maxV=maxVLower;
            end
%                     thetaNext=angThres+5;
        end
    else %when size of path is 1 (direct line of sight    
        if (devFor>radiusAccWP && dis>normSegLenThresh &&devBack>radiusAccWP&&cmdV>0)
            maxV=maxVUpper;
        else
            maxV=maxVLower;
        end
    end
    
%     if dis<(2*bloatFactor+0.2) %hard condition for all small line segs we go very slow
%         maxV=maxVSlow;
%     end
    
    if (hardBrake || maxV==maxVUpper)
        if (maxV==maxVUpper && devFor<radiusAccWP/2) %slow down from max upper in 2 stages
           maxV=maxVLower;
        end
        if devFor<radiusAccWP/3 %brake before stay away length segmeent
            maxV=maxVSlow;
        end
    end
    
    if size(Path,1)~=1
        if gotopt~=size(Path,1)
            if (dis<smallSegLenThresh&&thetaNext>angThres/3*2) %for u turn cases at the "vertex" of the u we want to slow down towards the end if the exit angle is greater than 45 degrees expressed here in terms of angthres
                if devFor<radiusAccWP/3
                    maxV=maxVSlow;
                end
            end
        end
    end
    
    
            
end