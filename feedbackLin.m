function [v,w] = feedbackLin(Vx,Vy,theta,Eps)

% Transform Vx and Vy commands (wrt inertial frame) into corresponding v
% and w commands using the feedback linearization technique presented in class 
% for a differential drive robot.
% 
%   INPUTS
%       Vx      given velocity in the x-direction wrt inertial frame (m/s)
%       Vy      given velocity in the y-direction wrt inertial frame (m/s)
%       theta   given angle between the inertial and body frame
%       Eps     epsilon, a value that is arbitrarily chosen
%   OUTPUTS
%       v        forward velocity (m/s)
%       w        angular velocity (rad/s)


Eps_matrix = [1 0; 0 1/Eps];

R_BI = [cos(theta) sin(theta); -sin(theta) cos(theta)];

vec = Eps_matrix*R_BI*[Vx; Vy];
v = vec(1);
w = vec(2);

end