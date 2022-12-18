 function[dataStore] = finalCompetition(Robot,maxTime)

%   Do not load the map to run this function
%   dataStore = RobotVisitingWaypoints(CreatePort,DistPort,TagPort,tagNum,maxTime) runs 
% 
%   INPUTS type
%       CreatePort  Create port object (get from running RoombaInit)
%       DistPort    Depth ports object (get from running CreatePiInit)
%       TagPort      Tag ports object (get from running CreatePiInit)
%       tagNum      robot number for overhead localization
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots


% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 10000;
end

try 
    % When running with the real robot, we need to define the appropriate 
    % ports. This will fail when NOT connected to a physical robot 
    CreatePort=Robot.CreatePort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort = Robot;
end

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', [], ...
                   'particles',[],...
                   'wp',[],...
                   'optWallsYes',[],...
                   'optWallsNo',[],...
                   'pose',[],...
                   'map',[],...
                   'G',[]);

% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('compMap.mat');
figure;
for i = 1:size(map,1)
    w = plot([map(i,1),map(i,3)],[map(i,2),map(i,4)],'k','LineWidth',2);
    hold on
end
xlabel('x (m)')
ylabel('y (m)')
hold all
% load('map2_4credits.mat');

%%%%% first map %%%%%
% load('map1_4creditsModified2.mat');
% load('map1_4creditsLessWP.mat');
% load('map1_check.mat');

%%%%% second map %%%%%
% load('map2_check.mat');
% load('map2_modified.mat');
% load('map2_4credits_removedOpt.mat');
% load('map2_edgecase.mat');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% color's rgb values
purple = [0.4940, 0.1840, 0.5560];
purpleL = purple  +0.4;
darkG = [0, 0.5, 0];
crimson = [0.6350, 0.0780, 0.1840];
darkY = [0.9290, 0.6940, 0.1250];
teal = [0 0.75 0.75];

%%%%%%%%%%%%%%%%%

% whichLED = 3;
% setting LED lights
% SetLEDsRoomba(CreatePort,whichLED,0,100); % LED is green
reached = 0;
resetLED = 0;

%%%%
originalMap = map;
% clubbing all optional walls into environment walls
map = [map;optWalls];
% storing waypoints
wptemp = [waypoints;ECwaypoints];
% radius of the robot
radius = 0.2;
% bloating/buffering all walls by bloatfactor
bloatFactor = radius  + 0.05;
distPt2Wall = bloatFactor + 0.07;
% resulting vertices of all polygons in environment 
obstacleVerts = polygonalWorld(map,stayAwayPoints,bloatFactor);
% computing dummy waypoints near eac optional wall
optWallsStruct = createDummyWP(optWalls,distPt2Wall,bloatFactor,map,stayAwayPoints,obstacleVerts);
% extracting dummy wapoints from above output
dp = optWallsStruct.wp; % n x 4
% reshape dp to 2n x 2 format
dp = reshape(dp',2,size(dp,1)*2)';
% fusing all waypoints in a vector 'wp'
wp = [wptemp];
for i = 1:size(wp,1)
   wayP = plot(wp(i,1),wp(i,2),'.','color',purple,'MarkerSize',25); 
end
% initial goalList
goalList = wp;
% goal list of dummy wp
goalListDP = dp;
% keepting track of which opt walls is not checked
optWallsNotChecked = optWallsStruct.walls;
% creating roadmap
disp('Creating roadmap...')
if isfile('roadmapFINAL.mat')    % if already ran roadmap once
    load('roadmapFINAL.mat');
else
    G = createRoadmap(obstacleVerts,wp,0);
    save('roadmapFINAL.mat', 'G');
end
dataStore.G = G;
disp('Done creating roadmap')
% test = dataStore.G;
%%%%%%% PLOT %%%%%%%%%%
% for i = 1:size(map,1)
%     w = plot([map(i,1),map(i,3)],[map(i,2),map(i,4)],'k','LineWidth',2);
%     hold on
% end
% hold on
% for i = 1:length(obstacleVerts)
%     pnew = polyshape(obstacleVerts{i}(1:end-1,1), obstacleVerts{i}(1:end-1,2));
%     obst = plot(pnew,'FaceColor','k','FaceAlpha',0.05);
%     hold on
% end
% hold on
% for i = 1:size(stayAwayPoints,1)
%    plot(stayAwayPoints(i,1),stayAwayPoints(i,2),'r.','MarkerSize',5);
%    hold on
% end
% hold on
% % edges
% for i = 1:size(test.E,1)
%     if i == 1
%         a = plot([test.E(i,1),test.E(i,3)],[test.E(i,2),test.E(i,4)], '--','color','g', 'LineWidth',1,'DisplayName','edges');
%     else
%         a = plot([test.E(i,1),test.E(i,3)],[test.E(i,2),test.E(i,4)], '--','color','g', 'LineWidth',1,'HandleVisibility','off');
%     end
% end
% % nodes
% for i = 1:size(test.V,1)
%     if i == 1
%         a = plot(test.V(i,1),test.V(i,2), 'm.','MarkerSize',10,'LineWidth',2,'DisplayName','nodes');
%     else
%         a = plot(test.V(i,1),test.V(i,2), 'm.','MarkerSize',10,'LineWidth',2,'HandleVisibility','off');
%     end 
%     
% end

%%%%%%
% checking which waypoints are no Qfree excluding dp
wp_notFree = checkWPinFree(wptemp, obstacleVerts);
% boolean variable to keep track of initial localization
initialized = 0;
% keeping track of which waypoints are visited in ANY path 
% gotopt will be reset after visiting each waypoint
gotopt = 1;
% variable to keep track of angle turnes
totalAng = 0;
% constants for visitWaypoints()
closeEnough = 0.1;
alpha = 1;
epsilon = 0.2;
wheel2center = 0.13;
maxV = 0.2;
% keep track of which optional walls is not checked
% optWallsNotChecked = optWallsStruct.walls; % n x 5 with first column being id
% bloat factor for mapping of optional walls
bloatFacAlong = 0.2;
bloatFacNormal = 0.1;
% creating cells for optional walls and regular walls
[vertsOptWalls,pgon] = optWallsGridVert(optWalls,bloatFacAlong,bloatFacNormal);
[vertsRegWalls,~] = optWallsGridVert(originalMap,bloatFacAlong,bloatFacNormal);
plot(pgon,'FaceColor','k','FaceAlpha',0.2);
hold on
% initial log odds depth for mapping of optional walls
L0 = 0;
occ_gridDepth = zeros(1,size(optWalls,1));

% sensor origin in robot frame
sensorOrigin = [0.13;0]; 
% sensorOrigin = [0;0];
% FOR PARTICLE FILTER
noOfParticles = 500;     
noOfParticlesAFTER = 30;   
% width of square centered at waypoint in which particles will be initialized
squareWidth = 0.2; 
% HARDCODING: No of depth meas
N = 9;
% process and measurement noise
R = 1e-3*eye(3); % process noise
Qdepth = 7e-3*eye(N); % himani: 1e-2
qbeac = 2e-3; %%%%% amy original: 2e-2
% Field of view for depth sensor
FOV = linspace(deg2rad(27),deg2rad(-27),9);

%%%%%% FOR BACKUPBUMP%%%%%%%%
fwdVel = 0.2; angVel = 0;
[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2center);
% initialize values
totaldist = 0;
totalang = 0;
bumpSensor = 0;
distReached = 0;
% parameters
bumpDist = -0.25;
bumpAng = deg2rad(0); % 30 deg clockwise  
bumpNum = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%
%Speed parameters
angThres = 60; %degrees
maxVUpper = 0.2;
maxVLower = 0.2;
maxVSlow = 0.1;
radiusAccWP = 1;
normSegLenThresh = 1.5;
highVelSegLenThresh = 0.5;
smallSegLenThresh = 2*bloatFactor*1.18;
%%%%%%%%%%%%%%%%%%%%%
counter = 0;
resetParticles = 0;
probList = 0.5*ones(size(optWalls,1),1);
threshold = 0.1;  % 0.04

% dummy wp stuff
pathUpdated = 0;
DPchecked = 0; % if all optional walls have been updated 
checkDPworth = 0;
angDeviation = 5;
lastSegLenThresh  = 0.1;
thetaToTurnThresh = 10;

% checking which EC is near optional walls
minLenWPToOptWall = 0.1;
ECnearOptWalls = checkECWPnearOptWall(ECwaypoints,optWallsStruct,minLenWPToOptWall);
% initial goal list = all waypoints in wp except ECnearOptWalls
if~isempty(ECnearOptWalls)
    for i = 1:size(ECnearOptWalls,1)
        idxEC = find(ismember(goalList, ECnearOptWalls(i,2:end), 'rows'));
        if ~isempty(idxEC)
            goalList(idxEC,:) = [NaN NaN];
        end
    end
    goalList = rmmissing(goalList);
end
switchList = 0; % switch from regular wp to dp when goalList is empty

% delay variable
u_last = 0;
[noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);

wp_notFreeDP = checkWPinFree(dp, obstacleVerts);

tic
while toc < maxTime
       
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    counter = counter+1;  
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ taking into account delays~~~~~~~~~~~~~%
%     if counter ~=1
        time = dataStore.odometry(end, 1) - dataStore.odometry(end-1, 1);
        delay = dataStore.rsdepth(end, 2);
        u_t = u_last + dataStore.odometry(end, 2:3) / time * (time - delay);
        u_last = dataStore.odometry(end, 2:3) / time * delay;
%     end
     %%%%%%%%%%%%%%%%%%%%%% CALL BACKUPBUMP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           
    [totaldist,totalang,bumpSensor,distReached,cmdV,cmdW,bumpNum] = backUpBump(dataStore,...
                totaldist,totalang,bumpSensor,distReached,bumpDist,bumpAng,cmdV,cmdW,...
                fwdVel,angVel,maxV,wheel2center,bumpNum); 
  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if counter==1    
        % only used for initializing PF
       [flattenedParticles,totalNOP]=pfInitializer (waypoints,squareWidth,noOfParticles,0,0,0,0);
       dataStore.particles.np = [toc totalNOP];
       dataStore.particles.x = [toc flattenedParticles];
       % assigning random initial weights
       initial_wt = ones(dataStore.particles.np(end,2),1)/dataStore.particles.np(end,2);
       dataStore.particles.w = [toc initial_wt'];
       disp('Initializing...')
    end 
    
    %%%%%%%%%%%%%%%%%%%%%%%%%% LOCALIZATION: PF %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % extracting NoOfParticles, each particle's state & weight
    % no of particles in particleSet@t-1 (pset_tm1)
    pset_tm1.np = dataStore.particles.np(end,2);
    access_x = dataStore.particles.x(end,2:end);
    pset_tm1.x = reshape(access_x,[3,pset_tm1.np]);
    pset_tm1.w = dataStore.particles.w(end,2:end);               
    % extracting control commands from dataStore/odometer
%     u_t = [dataStore.odometry(end,2);dataStore.odometry(end,3)];  
    % extracting measurements from dataStore/rsdepth/beacon
    ztdepth = dataStore.rsdepth(end,3:end)';
    % setting depth meas to NaN if out of bounds
    for i = 1:length(ztdepth)
        if(ztdepth(i)<=0.2||ztdepth(i)>=10)
            ztdepth(i) = NaN; 
        end
    end  
    % extracting beacon measurements
    ztbeacon = [];
    id = [];
    if ~isempty(dataStore.beacon)
        for i = 1:size(dataStore.beacon,1)
            ztbeacon = [ztbeacon;dataStore.beacon(i,4:5)'];
            % extracting id of the beacon from dataStore;
            id = [id;dataStore.beacon(i,3)];
        end
        % changing Qbeacon (size of it is dynamic)
        Qbeacon = qbeac*eye(length(ztbeacon));
        Q = blkdiag(Qdepth,Qbeacon);
        % fusing depth and beacon meas
        zt = [ztdepth;ztbeacon];  
        dataStore.beacon = [];
    else
        Q = Qdepth;
        zt = ztdepth;
    end

    
    % calling PF 
    pset_t = PF(pset_tm1,map,sensorOrigin,...
                u_t,zt,R,Q,@g,@hDepthBeacon,id,FOV,beaconLoc);

    % store particle data
    dataStore.particles.x = [dataStore.particles.x;toc ...
        reshape(pset_t.x,[1,size(pset_t.x,1)*size(pset_t.x,2)])];
    dataStore.particles.w = [dataStore.particles.w;toc pset_t.w];
    dataStore.particles.np = [dataStore.particles.np;toc pset_t.np];
    
    % estimate pose from the particle set
    if ~initialized
        estPose = PsetToPose(pset_t,10);
    else
        estPose = PsetToPose(pset_t,3);
    end
%     estPose = dataStore.truthPose(end,2:end);
    dataStore.pose = [dataStore.pose;estPose]; 
    currentTheta = rad2deg(dataStore.pose(end,3));
    currentPose = [dataStore.pose(end,1:2) currentTheta];
    %%%%%%%%%%%%%%%%%%%%%%% END OF LOCALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    %%%%%%%%%%%%%%%%%%%% INITIALIZE & MOTION PLANNING  %%%%%%%%%%%%%%%%%%%
    if ~initialized
        %~~~~~~~~~~~~~~~~~~~~~~~ INITIALIZATION  ~~~~~~~~~~~~~~~~~~~~~~~~~%
        % turn in place
        cmdV = 0; cmdW = 0.2;       
        totalAng = totalAng + dataStore.odometry(end,3);
        checkFinal = 0;
        checkEmpty = 0;
        if totalAng >= deg2rad(360)
            Init = [currentPose(1:2) wrapTo360(rad2deg(currentPose(3)))]
            disp('Initialized')
            % setting LED 
            BeepRoomba(CreatePort);
%             SetLEDsRoomba(CreatePort,whichLED,100,100);    % visited the wp, LED red 
            reached = 1;
            resetLED = 1;
            % stop the robot
            SetFwdVelAngVelCreate(CreatePort, 0, 0 );
            start = closestWP(dataStore.pose(end,1:2),waypoints);
            resetParticles  = 1;
            % remove starting wp from goal list
            [res,loc] = ismember(start,goalList,'rows');
            if res
                goalList(loc,:) = [];
            end
            % store wp as start is found
            dataStore.wp = [dataStore.wp;start];
            % find next wp and path to visit it from start location
            [goal,Path] = findNextWP(G,goalList,start);
            pathPlot = plot(Path(:,1), Path(:,2),'color',darkY,'LineWidth',2)  ;
            Path = Path(2:end,:);
            initialized = 1;
        end
    else
        if resetParticles             
            % sort the weights
            [~,I] = sort(dataStore.particles.w(end,2:end),2,'descend');
            dataStore.particles.w = [toc dataStore.particles.w(end,I(1:noOfParticlesAFTER)+1)];
            access_x = dataStore.particles.x(end,2:end);
            tempX = reshape(access_x,[3,dataStore.particles.np(end,2)]);
            tempX = tempX(:,I(1:noOfParticlesAFTER));
            dataStore.particles.x = [toc reshape(tempX,[1,size(tempX,1)*size(tempX,2)])];
            dataStore.particles.np = [toc noOfParticlesAFTER];
%            [flattenedParticles,totalNOP]=pfInitializer (start,squareWidth,noOfParticlesAFTER,1,0,0,dataStore.pose(end,3));
%            dataStore.particles.np = [[];toc totalNOP];
%            dataStore.particles.x = [[];toc flattenedParticles];
           % assigning uniform initial weights
%            initial_wt = ones(dataStore.particles.np(end,2),1)/dataStore.particles.np(end,2);
%            dataStore.particles.w = [[];toc initial_wt'];
        end
        resetParticles = 0;   
        %~~~~~~~~~~~~~~~~~~~~~~~~~ AFTER INITIALIZATION ~~~~~~~~~~~~~~~~~~%  
        
        % optWallsUpdated := each row is of the format:[id x1 y1 x2 y2 occ]  
        occ_gridDepth = zeros(1,size(optWalls,1));
        occ_gridDepth = logOddsDepth(dataStore.pose(end,1:end),dataStore.rsdepth(end,3:end)...
                        ,occ_gridDepth, L0, vertsOptWalls,vertsRegWalls,sensorOrigin);
        occ_grid = 1-1./(1 + exp(occ_gridDepth));
        
        [optWallsUpdated ,probList]= updateWall(occ_grid, optWallsStruct, optWalls,optWallsNotChecked,probList,threshold);
        % if any optional wall info is updated and that wall has not 
        % been updated unitl now then,
         if ~isempty(optWallsUpdated)
            for i = 1:size(optWallsUpdated,1)
                % store that this wall is checked
                optID = optWallsUpdated(i,1);
                % boolean occ = whether this optwall is there or not
                occ = optWallsUpdated(i,end); 
                [res,loc] = ismember(optID,optWallsNotChecked(:,1),'rows');
                if res && occ ~= 0.5     % wall to check if not in optWallsNotChecked and probability has changed enough
                    optWallsNotChecked(loc,:) = [];
                    
                elseif isempty(res)
                    % break out of the loop if this optWall is already updated
                    break;
                end                
                if occ == 1   
                    disp('updating optional wall to be there')
                   % store this optional wall
                    dataStore.optWallsYes = [dataStore.optWallsYes;...
                                            optWallsUpdated(i,2:end-1)];                
                elseif occ == 0   
                    disp('updating optional wall to NOT be there')
                    % store that this optional wall is not +nt
                    dataStore.optWallsNo = [dataStore.optWallsNo;...
                                            optWallsUpdated(i,2:end-1)];
                    % update the map such that these optional walls are removed
                    [res,loc] = ismember(optWallsUpdated(i,2:end-1),map,'rows');
                    if res
                       map(loc,:) = []; 
                    end
                    obstacleVerts = polygonalWorld(map,stayAwayPoints,bloatFactor);
                end
                if occ ~= 0.5 
                     % add ECnearOptWall to goalList
                    [res,loc] = ismember(optID,ECnearOptWalls(:,1),'rows');
                    if res
                        wpNum = find(ismember(ECnearOptWalls(:,2:end),ECnearOptWalls(loc,2:end), 'rows'));
                        % if waypoint only appears once (only near one
                        % optional wall)
                        if ~isempty(wpNum)
                            if length(wpNum) == 1
                                goalList = [goalList; ECnearOptWalls(loc,2:end)];
                            end
                        end
                        % remove waypoint from list
                        ECnearOptWalls(loc,:) = [];
                    end
                    [~, wp_update] = removeWaypoints([goalList; dp],...
                                    optWallsUpdated(i,2:end-1),optWallsStruct);
                    % update the roadmap based on this info
                    % stop the robot
                    SetFwdVelAngVelCreate(CreatePort, 0, 0 );
                    disp('updating map')
                    G = updateRoadmap(G, wp_update,occ, obstacleVerts);
                    G = updateEnclosedRoadmap(G,obstacleVerts,[wp_notFree;wp_notFreeDP],wp_update,occ);
                    dataStore.G = G;
                    % update which waypoints are enclosed 
                    wp_notFree = checkWPinFree(goalList, obstacleVerts);
                                   
                    % remove dummy wps from wp_notFree
                    if ~isempty(wp_notFree)
                        for w = 1:size(wp_update,1)
                            [res,loc] = ismember(wp_update(w,:),wp_notFree,'rows');  % remove dp from goalList
                            if res
                                wp_notFree(loc,:) = [NaN NaN];
                            end
                        end
                        wp_notFree = rmmissing(wp_notFree);
                    end
                                       
                    % remove dummy wps from goalList
                    if ~isempty(goalList)
                        for w = 1:size(wp_update,1)
                            [res,loc] = ismember(wp_update(w,:),goalList,'rows');  % remove dp from goalList
                            if res
                                goalList(loc,:) = [NaN NaN];
                            end
                        end
                        goalList = rmmissing(goalList);
                    else
                        disp('goalList is empty,1')
                    end
                                       
                    % also remove dummy wps from goalListDP
                    for w = 1:size(wp_update,1)
                        [res,loc] = ismember(wp_update(w,:),goalListDP,'rows');  % remove dp from goalList
                        if res
                            goalListDP(loc,:) = [NaN NaN];
                        end
                    end
                    goalListDP = rmmissing(goalListDP);
                    
                    % ~~~~~~~~ remove dummy waypoints that are currently in the path ~~~~~~~~
                    
                    % start path as next waypoint we're already trying to reach
                    for p = 1:size(Path, 1)
                        for w = 1:size(wp_update,1)
                            idxP = find(ismember(Path(p,:), wp_update(w,:), 'rows'),1);
                            if ~isempty(idxP)   % dummyWP in path
                                pathUpdated =1;
                                break;
                            end
                        end
                        if pathUpdated
                            break;
                        end
                    end

                    % ~~~~~~~~~~~~~~~~~~~~UPDATING PATH W/ DUMMY WP ~~~~~~~~~~~~~~~~~~~~~~
                    if pathUpdated    % if path has been updated due to removing dummy WP
                        if gotopt == 1
                            start = Path(gotopt,:);
                        else
                            start = Path(gotopt-1,:);   % start is previously visited waypoint
                        end
                                                
                        % remove dummy wp we're currently on our way to
                        if ~isempty(goalList)
                            [res,loc] = ismember(Path(gotopt,:),goalList,'rows');  % remove dp from goalList
                            if res
                                goalList(loc,:) = [];
                            end
                        else
                            disp('goalList is empty,2')
                        end
                        
                        % also remove from goalListDP
                        [res,loc] = ismember(Path(gotopt,:),goalListDP,'rows');  % remove dp from goalList
                        if res
                            goalListDP(loc,:) = [];
                        end
                                                
                        % reset gotopt
                        gotopt = 1;
                        checkFinalP = isequal(sortrows(goalList),sortrows(wp_notFree));
                        checkEmptyP = (isempty(goalList) && isempty(wp_notFree));

                        if ~checkFinalP && ~checkEmptyP
                            [goal,Path] = findNextWP(G,goalList,start);
                            [res,~] = ismember(goal,goalListDP,'rows');    % check if dummy or if EC
                            if DPchecked && res  % check for parallel paths
                                worthIt = checkParallelOptWall(Path,optWallsStruct,angDeviation,start,lastSegLenThresh,thetaToTurnThresh);
                                count = 0;
                                goalLength = size(goalListDP,1);
                                goalRemoved = [];
%                                 worthIt = 1;
                                while ~worthIt && count <= goalLength   % don't go to dummy wp
                                    [~,loc] = ismember(goal,goalList,'rows');
                                    goalRemoved = [goalRemoved; goal];
                                    goalList(loc,:) = [];
                                    goalList_notfree = checkWPinFree(goalList, obstacleVerts);
                                    checkF = isequal(sortrows(goalList),sortrows(goalList_notfree));
                                    if isempty(goalList) || checkF
                                        checkDPworth = 1;
                                        Path = start;
                                        break
                                    end
                                    [goal,Path] = findNextWP(G,goalList,start);
                                    count = count + 1;
                                    worthIt = checkParallelOptWall(Path,optWallsStruct,angDeviation,start,lastSegLenThresh,thetaToTurnThresh);
                                end

                                % add the goalremoved back into goalList          
                                if ~checkDPworth
                                   goalList = [goalList; goalRemoved];
                                else
                                   goalList = [];
                                end
                            end
                            
                        end

                        delete(pathPlot)
                        pathPlot = plot(Path(:,1), Path(:,2),'color',darkY,'LineWidth',2)  ;                        
%                         Path = Path(2:end,:);
                        pathUpdated = 0;

                    end           
                    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~                   
                end
            end
         end
               
        % find velocity commands using visitWaypoints()
        if ~bumpSensor
            %@@@@@@@@@@@@@@@@@@@@@@@ SPEED CONTROL @@@@@@@@@@@@@@@@@@@@@@@@
%             if gotopt~=1
%                 dis = norm([(Path(gotopt-1,1)-Path(gotopt,1));(Path(gotopt-1,2)-Path(gotopt,2))],2);
%             else
%                 dis = norm([(start(1)-Path(gotopt,1));(start(2)-Path(gotopt,2))],2);
%             end
%             if dis >=2
%                 maxV = 0.3;
%             else
%                 maxV = 0.2;
%             end
            %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        
            currentPose4Vel = dataStore.pose(end,1:2);               

            maxV=speedControl(currentPose4Vel,gotopt,Path,start,angThres,...
                radiusAccWP,maxVUpper,maxVLower,maxVSlow,smallSegLenThresh,...
                normSegLenThresh,highVelSegLenThresh,cmdV);
        
            %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 
            [cmdV,cmdW,gotopt] = visitWaypoints(Path,dataStore.pose(end,1:end),...
                                gotopt,epsilon,closeEnough,alpha);
            [cmdV,cmdW] = limitCmds(cmdV,cmdW,maxV,wheel2center);
        end
         
        % if the goal for this path is reached
        % if goalList is empty, change to goalListDP
        checkFinalDP = isequal(sortrows(goalList),sortrows([goal;wp_notFree]));
        checkEmptyDP = (isempty(goalList) && isempty(wp_notFree));
        if isequal(goal,waypoints(1,:)) && gotopt==size(Path,1)+1
           goalList; 
        end
        if (isequal(goalList,goal) || checkFinalDP || checkEmptyDP) && ~DPchecked
            goalList = [goalListDP;wp_notFree];
            switchList  = 1;
        end
        if gotopt==size(Path,1)+1 && ~isempty(goalList)
            % setting LED 
            [res,~] = ismember(goal,wp,'rows');
            if res
                BeepRoomba(CreatePort);
%                 SetLEDsRoomba(CreatePort,whichLED,100,100);    % visited the wp,red
                reached = 1;
                resetLED = 1;
            end         
            % find next goal and path to  visit it
            start = goal;  
            % remove starting wp from goal list
            [res,loc] = ismember(start,goalList,'rows');
            if res
                goalList(loc,:) = [];
            end
            
            % remove starting wp from goal list if its a dummy wp
            [res,loc] = ismember(start,goalListDP,'rows');
            if res
                goalListDP(loc,:) = [];
            end
            
            % if goalList is empty, change to goalListDP
            checkFinalDP = isequal(sortrows(goalList),sortrows(wp_notFree));
            checkEmptyDP = (isempty(goalList) && isempty(wp_notFree));

            if (checkFinalDP || checkEmptyDP || switchList) && ~isempty(goalListDP) 
%                 goalList = [goalListDP; wp_notFree];
                DPchecked = 1;
                % create map with dummy wp
                SetFwdVelAngVelCreate(CreatePort, 0,0);
                plot(dp(:,1),dp(:,2),'k*')
                hold on;
                disp('creating roadmap for optional walls...')
                G = createRoadmap(obstacleVerts,[goalList; start;ECnearOptWalls(:,2:end)],0);
                dataStore.G = G;
                disp('done')
                switchList = 0;
            end

            if isempty(goalListDP)
                SetFwdVelAngVelCreate(CreatePort, 0, 0)
                DPchecked = 1;
            end

            % store wp as start is found
            if ismember(start,wp,'rows')
                dataStore.wp = [dataStore.wp;start];
            end
            checkFinal = isequal(sortrows(goalList),sortrows(wp_notFree));
            checkEmpty = (isempty(goalList) && isempty(wp_notFree));

            if ~checkFinal && ~checkEmpty
                [goal,Path] = findNextWP(G,goalList,start);
                
                [res,~] = ismember(goal,goalListDP,'rows');    % check if dummy or if EC
                if DPchecked && res % check for parallel paths
                    worthIt = checkParallelOptWall(Path,optWallsStruct,angDeviation,start,lastSegLenThresh,thetaToTurnThresh);
                    count = 0;
                    goalLength = size(goalListDP,1);
                    goalRemoved = [];
%                     worthIt = 1;
                    while ~worthIt && count <= goalLength  % don't go to dummy wp
                        [~,loc] = ismember(goal,goalList,'rows');
                        goalRemoved = [goalRemoved; goal];
                        goalList(loc,:) = [];
                        goalList_notfree = checkWPinFree(goalList, obstacleVerts);
                        checkF = isequal(sortrows(goalList),sortrows(goalList_notfree));
                        if isempty(goalList) || checkF
                            checkDPworth = 1;
                            Path = start;
                            break
                        end
                        [goal,Path] = findNextWP(G,goalList,start);
                        count = count + 1;
                        worthIt = checkParallelOptWall(Path,optWallsStruct,angDeviation,start,lastSegLenThresh,thetaToTurnThresh);
                    end
                    
                    % add the goalremoved back into goalList
                    if ~checkDPworth
                       goalList = [goalList; goalRemoved];
                    else
                        goalList = [];
                    end
                    
                end
                    
            end
            delete(pathPlot)
            pathPlot = plot(Path(:,1), Path(:,2),'color',darkY, 'LineWidth',2);
            Path = Path(2:end,:);
            gotopt = 1; % rest gotopt             
        end
        
        if reached 
            BeepRoomba(CreatePort);
%             resetLED = resetLED+1;
%             if resetLED > 10
%                 SetLEDsRoomba(CreatePort,whichLED,0,100);
                resetLED = 0;
                reached = 0;
%             end
        end
        %~~~~~~~~~~~~~~~~ IF TIME IS UP OR MISSION COMPLETED ~~~~~~~~~~~~~%
        if (isempty(goalList)||checkFinal || checkEmpty) && DPchecked  
            % output  dataStore.wp
            visitedWaypoints = dataStore.wp;
            % plot the current map in black
            figure;
            map = [originalMap;dataStore.optWallsYes];
            for i = 1:size(map,1)
                w = plot([map(i,1),map(i,3)],[map(i,2),map(i,4)],'k','LineWidth',2);
                hold on
            end
            hold on
            % plot the optional walls that hasn't been checked in red
            if ~isempty(optWallsNotChecked)
            for i = 1:size(optWallsNotChecked,1)
               notChecked = plot([optWallsNotChecked(i,2),optWallsNotChecked(i,4)],...
                            [optWallsNotChecked(i,3),optWallsNotChecked(i,5)]...
                            ,'r','LineWidth',2); 
               hold on
            end
            end
            hold on
            % plot the trajectory
            traj = plot(dataStore.pose(:,1),dataStore.pose(:,2),'b.');
            for i = 1:size(visitedWaypoints,1)
                visitedWP = plot(dataStore.wp(i,1),dataStore.wp(i,2),'mo',...
                    'MarkerSize',15,'LineWidth',2);
                hold on
            end
            for i = 1:size(wp,1)
               wayP = plot(wp(i,1),wp(i,2),'.','color',purple,'MarkerSize',25); 
            end
            if ~isempty(optWallsNotChecked)
                legend([notChecked w traj visitedWP,wayP],'optWalls not checked',...
                    'updated map','trajectory','visited waypoints','waypoints');
            else
                legend([w traj,visitedWP,wayP],'updated map','trajectory',...
                    'visited waypoints','waypoints');
            end
            xlabel('x (m)')
            ylabel('y (m)')
            axis equal
            title('Final plot')
            break;
        end  
    end   
    
    % plot
    if maxV==maxVLower
        plot(dataStore.pose(end,1),dataStore.pose(end,2),'m.')
    elseif maxV==maxVUpper
        plot(dataStore.pose(end,1),dataStore.pose(end,2),'m.')
    else 
        plot(dataStore.pose(end,1),dataStore.pose(end,2),'.','color',teal)
    end
    dataStore.map = map;
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    else
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
    end
%     pause(0.1);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );
