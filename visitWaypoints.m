function [v,w, gotopt_next] = visitWaypoints(wp, pose, gotopt, Eps, closeEnough, alpha)

% Calculate the desired [v, w] controls to drive the robot along a
% series of waypoints.
% 
%   INPUTS
%       wp              nx2 matrix of waypoints, where each row is the (x,y)
%                       coordinate of a waypoint
%       pose            robot's current pose [x y theta]  (1-by-3) 
%       gotopt          index of the waypoint being reached
%       Eps             epsilon, a value that is arbitrarily chosen
%       closeEnough     acceptable amount of deviation from the waypoint
%       alpha           scaling coefficient to convert position into Vx and Vy 
% 
%   OUTPUTS
%       v               forward velocity (m/s)
%       w               angular velocity (rad/s)
%       gotopt_next     next index of waypoint that should be used


gotopt_next = gotopt;
wp_x = wp(gotopt,1);
wp_y = wp(gotopt,2);

xpos = pose(1);
ypos = pose(2);
theta = pose(3); 

dev = sqrt((wp_x - xpos).^2 + (wp_y - ypos).^2);   % deviation from waypoint

% check if can move onto next waypoint (if current waypoint is reached)
if dev <= closeEnough && gotopt < size(wp,1)    
    gotopt_next = gotopt+1;
    wp_x = wp(gotopt_next,1);
    wp_y = wp(gotopt_next,2);
end


Vx = alpha*(wp_x - xpos);
Vy = alpha*(wp_y - ypos);

[v,w] = feedbackLin(Vx, Vy, theta, Eps);

if dev <= closeEnough && gotopt == length(wp)
    v = 0; w = 0;
    gotopt_next = gotopt+1;
end
end
