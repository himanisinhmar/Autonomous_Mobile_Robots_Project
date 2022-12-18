function expMeas = hDepthBeacon(Xprev,map,sensorO,FOV,id,beaconLoc)

    % compute expected depth meas
    range = rangePredict(Xprev, map, sensorO, FOV);
    depth = depthPredict(range,FOV);
    
    % compute expected beacon location
    if ~isempty(id)
        beacon = [];
        for i = 1:length(id)
            idx = find(beaconLoc(:,1)==id(i));
            beaconPose = beaconLoc(idx, 2:3)';
            beaconPose = global2robot(Xprev, beaconPose);   % convert beacon location into sensor frame

            x_dist = beaconPose(1) - sensorO(1);
            y_dist = beaconPose(2) - sensorO(2);
            beacon = [beacon;x_dist;y_dist];
        end
        expMeas = [depth;beacon];
    else
        expMeas = depth;
    end
    
    
    
end

