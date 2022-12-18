function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,WHEEL2CENTER) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       wheel2Center distance from the wheels to the center of the robot(in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   FANG, AMY 

% find velocities of wheels
wheelVel1 = fwdVel + wheel2Center*angVel;    % v = R*omega
wheelVel2 = fwdVel - wheel2Center*angVel;

wheelVels = [wheelVel1 wheelVel2];
wheelVelmax = max(abs(wheelVel1), abs(wheelVel2));

% check if saturated
if wheelVelmax > maxV
    wheelVels = wheelVels * maxV/wheelVelmax;
end

% parse out forward and ang velocities
cmdV = sum(wheelVels)/2;
cmdW = (wheelVels(1) - wheelVels(2))/(2*wheel2Center);

end
