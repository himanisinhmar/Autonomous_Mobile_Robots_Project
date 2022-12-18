function [G_t] = GjacDiffDrive(X_prev,d,phi)

% Input: state at t-1, controls at t = distance traveled and angle turned
% Output: linear matrix G_t

    theta = X_prev(3);
    % Case I: if phi != 0
    if(phi~=0)
        G_t = [1 0 (d/phi)*(cos(theta + phi) - cos(theta));
              0 1 (d/phi)*(sin(theta + phi) - sin(theta));
              0 0 1];
    else
        G_t = [1 0 -d*sin(theta);
               0 1 d*cos(theta);
               0 0 1];
    end

end

