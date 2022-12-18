function[xyG] = robot2global(pose,xyR)
% ROBOT2GLOBAL: transform a 2D point in robot coordinates into global
% coordinates (assumes planar world).
% 
%   XYG = ROBOT2GLOBAL(POSE,XYR) returns the 2D point in global coordinates
%   corresponding to a 2D point in robot coordinates.
% 
%   INPUTS
%       pose    robot's current pose [x y theta]  (1-by-3)
%       xyR     2D point in robot coordinates (1-by-2)
% 
%   OUTPUTS
%       xyG     2D point in global coordinates (1-by-2)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   LASTNAME, FIRSTNAME 

xpos = pose(1);
ypos = pose(2);
theta = pose(3);

T_IB = [cos(theta) -sin(theta) xpos;
        sin(theta)  cos(theta) ypos;
        0           0           1];

point_trans = T_IB*[xyR 1]';  % [point_I 1]' = T_IB*[point_B 1]'
xyG = point_trans(1:2)';

end
