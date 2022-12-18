function[range] = rangePredict(robotPose,map,sensorOrigin,angles)
% RANGEPREDICT: predict the range measurements for a robot operating
% in a known map.
%
%   RANGE = RANGEPREDICT(ROBOTPOSE,MAP,ROBOTRAD,ANGLES) returns
%   the expected range measurements for a robot operating in a known
%   map.
%
%   INPUTS
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
%       map         	N-by-4 matrix containing the coordinates of walls in
%                   	the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       angles      	1-by-K vector of the angular orientation of the range
%                   	sensor(s) in the sensor-fixed frame, where 0 points
%                   	forward. All sensors are located at the origin of
%                   	the sensor-fixed frame.
%
%   OUTPUTS
%       range       	K-by-1 vector of ranges (meters)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 2
%   SINHMAR, HIMANI

% Algorithm for finding Range
% 1) find the line equation of the laser beam emitted from the sensor
%    slope of this line wrt sensor frame = tan(angels(i))
% 2) Let one point on this line be the sensor frame origin(beam emitted
%    from here) = (sensorOrigin(1),sensorOrigin(2))
% 3) eqn of this line: y = sensorOrigin(2) + (x-sensorOrigin(1))*slope_sensorLine  
% 4) since intersectPoint.m needs two points on this line as input, let them
% be point 1: ((sensorOrigin(1),sensorOrigin(2))) 
% and ("choose any value of x : depending on how far senosr can see", "choose y acc. to above eqn then")
% 5) find closest obstacle at a given angle and then range = distance([robotPose(1),robotPose(2)],[x,y]"as returned from intersectPoint.m")

N = size(map,1);
k = max(size(angles,1),size(angles,2));
max_dist = 10;
% initialize range vector to have max distance values 
% values, in case the obstacle is at a greater than the distance that sensor can see
range = max_dist*ones(k,1);
x0 = sensorOrigin(1);y0 = sensorOrigin(2);
closest_distance = max_dist;
ua_old =1;
for i = 1:k
%     % option 1 to find second point coordinates in robot frame
%     
%     slope_sensorLine_R = tan(angles(i));
%     x_onSensorLine_R = sensorOrigin(1) + 6*max_dist;
%     y_onSensorLine_R = sensorOrigin(2) ...
%         + (x_onSensorLine_R - sensorOrigin(1))*slope_sensorLine_R;

    % option 2 to find second point coordinates in robot frame
    x_onSensorLine_R = sensorOrigin(1) + max_dist*cos(angles(i));
    y_onSensorLine_R = max_dist*sin(angles(i));
    
    for j = 1:N
        % converting both wall points to robot frame for each wall
        xyR1 = global2robot(robotPose,[map(j,1);map(j,2)]); 
        x1 = xyR1(1); y1 = xyR1(2);
        xyR2 = global2robot(robotPose,[map(j,3);map(j,4)]);
        x2 = xyR2(1); y2 = xyR2(2);
        [isect,x,y,ua] = intersectPointHimani(x0,y0,x_onSensorLine_R,...
                         y_onSensorLine_R,x1,y1,x2,y2);
        if (isect)
            distance =  sqrt((x-x0)^2 + (y-y0)^2);             
            if(distance < range(i))   
                range(i) = distance;
            else
                range(i) = range(i);
            end
        end
    end
       
end
end



