function estPose = PsetToPose(pset,avgParticles)
  
    % sorting particles by their weight
    [~,I] = sort(pset.w,2,'descend');
    % taking average of best 6 particles
    poseAvg = pset.x(:,I(1:avgParticles));
    estPose = sum(poseAvg,2)./avgParticles;
%     estPose = pset.x(:,I(1));
    estPose = estPose';
%     if estPose(3)>deg2rad(360)
%         estPose(3) = estPose(3)-deg2rad(360);
%     elseif estPose(3)<-deg2rad(360)
%         estPose(3) = estPose(3)+deg2rad(360);
%     end
    

end

