function [totaldist,totalang,bumpSensor,distReached,cmdV,cmdW, bumpNum] = backUpBump(dataStore,...
                totaldist,totalang,bumpSensor,distReached,bumpDist,bumpAng,cmdV,cmdW,...
                fwdVel,angVel,maxV,wheel2Center, bumpNum)

%     maxV = 0.1;
    % if bump is detected, move backwards
    if dataStore.bump(end,2)+dataStore.bump(end,3)+dataStore.bump(end,7) > 0
        
        bumpSensor = 1;
        cmdV = -maxV;        % reverse vel
        cmdW = 0;
    end
    
    % if bump, back up 0.25 m
    if bumpSensor 
        totaldist = totaldist + dataStore.odometry(end,2);
        
        % if reached -0.25m, stop and rotate
        if totaldist <= bumpDist     
            cmdV = 0;
            cmdW = bumpNum*0.2;
            
            totaldist = 0;   % reset totaldist for next bump
            distReached = 1;
        end
    end
    
    % if bump and backed up 0.25 m, turn 30 deg clockwise
    if bumpSensor && distReached
        totalang = totalang + dataStore.odometry(end,3);
        
         % if reached 30 deg, move forward as normal
        if abs(totalang) >= abs(bumpAng)    
            [cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center);
%             disp('RESETTING BUMP')
            % reset for next bump
            totalang = 0;       
            bumpSensor = 0;     
            distReached = 0;
            bumpNum = -bumpNum;
        end
        
    end

end

