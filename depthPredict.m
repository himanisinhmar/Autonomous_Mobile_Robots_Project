function [depth] = depthPredict(range,angles)
% this function outputs the depth from the closest obstacle using
% range and angles as input
    n = max(size(range,1),size(range,2));
    depth = zeros(n,1); % row vector

    for i = 1: n
       depth(i) = range(i)*cos(angles(i));
    end

%     depth = range.*cos(angles);
end

