function q_start=closestWP(particlePose,q_wp)
%     function q_start=closestWP(dataStore,q_wp)

%     [m,i]=max(dataStore.weights(end,:));
%     startIndex=((i-1)*3)+1;
%     particlePose=[dataStore.particles(end,startIndex) dataStore.particles(end,startIndex+1)];
    [n,j]=min(vecnorm((q_wp-particlePose)'));
    q_start=q_wp(j,:);

end