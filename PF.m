function particleSetT = PF(particleSetTm1,map,sensorO,...
                             ut,zt,R,Q,g,hDepthBeacon,id,FOV,beaconLoc)

% particleSetTm1.np   no. of particles at t-1(scalar)
% particleSetTm1.x    state of all particles at t-1(3 by np)
% particleSetTm1.w    weight of each particle at t-1(np by 1)
% 	map
% 	sensorOrigin      sensor origin on the robot, required for
%                     computing hdepth i..e for depthPredict.m
% 	ut                controls from odometry 
%                     i.e. distance traveled and angle turned
% 	zt                fused measurements: zt = [depthMeas;xbeacon;yBeacon]
% 	R                 process noise covariance matrix 
%   Q                 measurement noise covariance matrix
% 	g(xt-1, ut)       dynamics matrix or update matrix required for the 
%                     prediction step, function handle 
% 	h                 measurement function handle


%   OUTPUTS
%   particleSetT     particles set at time t. it is a struct with
%                    follwoing fields: x, w, np

%   HIMANI SINHMAR
%   CORNELL UNIVERSITY

% find min and max bounds of the map
map_maxX = max(max(map(:,1)),max(map(:,3)));
map_minX = min(min(map(:,1)),min(map(:,3)));
map_maxY = max(max(map(:,2)),max(map(:,4)));
map_minY = min(min(map(:,2)),min(map(:,4)));

x = particleSetTm1.x;
np = particleSetTm1.np;
expected_meas = zeros(length(zt),np);
% prediction step for all particles
for i = 1:np % for each particle update
    % updated particle Loc = mean + sigma*(random no from normal distribution)
    particleSetT.x(:,i) = feval(g,x(:,i),ut(1),ut(2)) + sqrt(R)*randn(3,1);
%     % compute expected (depth+beacon) meas by calling function handle h
%     expected_meas(:,i) = hDepthBeacon(particleSetT.x(:,i),map,sensorO,FOV,id,beaconLoc);
end

% UPDATE: only if meas are non empty
if(isempty(zt))
    particleSetT.w = particlesSetTm1.w;
    particleSetT.np = particlesSetTm1.np;
else  
    for i = 1:np
        % compute expected (depth+beacon) meas by calling function handle h
        expected_meas(:,i) = hDepthBeacon(particleSetT.x(:,i),map,sensorO,FOV,id,beaconLoc); 
    end
    % slice NaN measurements
    meas_sliced = []; indices = []; EM = [];Qt_temp = [];
    for m = 1:size(zt,1) % for each meas
        if (~isnan(zt(m)))
            meas_sliced = [meas_sliced;zt(m)];
            EM = [EM;expected_meas(m,:)];
            indices = [indices;m];
        end
    end
    for i = 1:size(indices,1)
        Qt_temp = [Qt_temp;Q(indices(i),indices(i))]; 
    end
    Qt = diag(Qt_temp); 

    % UPDATE: only if sliced meas are non empty
    if(isempty(meas_sliced))
        particleSetT.w = particleSetTm1.w;
        particleSetT.np = particleSetTm1.np;
    else
        % compute weight for all particles
        eta = 0;
        for i=1:np % for each particle compute weight
            em = EM(:,i);
            % normpdf for P(zt|xt) 
%             temp = 1;
%             for j = 1:size(indices,1) % for each meas compute prob.
%                 temp = temp*normpdf(meas_sliced(j),EM(j,i),sqrt(Qt(j,j)));
%             end   
            % mvnpdf for P(zt|xt)
            temp = mvnpdf(meas_sliced,em,sqrt(Qt));
            particleSetT.w(i) = temp; % P(zt|xt)
            % using actual formula fro mvnpdf
%             multiDistr = 1/(2*pi*sqrt(det(Qt)))*exp(-1/2*(em-meas_sliced)'*inv(Qt)*(em-meas_sliced));
%             particleSetT.w(i) = particleSetTm1.w(i) * multiDistr;
%              particleSetT.w(i) = multiDistr;
            
            if(particleSetT.x(1,i)>map_maxX||particleSetT.x(1,i)<map_minX...
               ||particleSetT.x(2,i)>map_maxY||particleSetT.x(2,i)<map_minY)
                particleSetT.w(i) = 0;
            end
            eta = eta + particleSetT.w(i);
        end
        % normalize weights
        for i=1:np % for each particle normalize the weight
            if(eta~=0)
                particleSetT.w(i) = particleSetT.w(i)/eta;
            else
%                 particleSetT.w = rand(np,1)'; % assign random weights
                particleSetT.w = (1/np)*ones(1,np); % assign uniform weights
%                 disp('all weights sum to 0')
            end               
        end
        % resampling
        replacement = true;
        idx = randsample(1:np,np,replacement, particleSetT.w);
        for i = 1:np % extract np new particles
            particleSetT.x(:,i) = particleSetT.x(:,idx(i));
            particleSetT.w(i) = particleSetT.w(idx(i));
        end
        particleSetT.np = np;
    end
end
end