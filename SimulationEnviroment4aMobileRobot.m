% ======================MAP CREATION===============

map = binaryOccupancyMap(100, 80, 1);
occ = zeros(80, 100);

occ(1,:) = 1; %Top
occ(end,:) = 1; %Bottom
occ([1:30, 51:80],1) = 1; %Left
occ([1:30, 51:80],end) = 1;%Right

%=========D==========
occ(30:49,21) = 1;
occ(30,21:25) = 1;
occ(31,26) = 1;
occ(32,27)=1;
occ(49,21:25) = 1;
occ(33:47,27) = 1;
occ(48,26)=1;

%=========E==========
occ(48,38:48) = 1;
occ(30:48,38) = 1;
occ(30,38:48) = 1;
occ(38,38:48) = 1;

%=========U==========
occ(30:50,60) = 1;
occ(50,60:67) = 1;
occ(30:50,67) = 1;

setOccupancy(map, occ)

figure
show(map)
title('DEU ROBOTICS')

robotRadius = 0.1;

mapInflated = copy(map);
inflate(mapInflated,robotRadius);

show(mapInflated)
title('DEU ROBOTICS')

%=====================PATH FINDING================

%=========== Starting Point of PATH KNWON CASE=========%
% initial_x = 15;
% initial_y = 10;
% 
% goal_x = 90;
% goal_y = 50;
% 
% 
% path = [initial_x initial_y;
%         15 40;
%         21 45;
%         15 70;
%         35 70;
%         40 70;
%         45 70;
%         48 70;
%         50 70;
%         90 70;
%         90 60;
%         goal_x   goal_y];
%     
% 
% flag = 0;
%  
% for i = 1:length(path)-1
%     if occ(path(i,2),path(i,1))==1
%         path(i,1) = path(i,1)-5;
%         path(i,2) = path(i,2)-5;
%         flag =1;
%     end
% end
%============END of PATH KNOWN CASE=======================%

% ====================Starting Point of PRM======================%

% prm = mobileRobotPRM(mapInflated,60); % assign 60 nodes on map
% startLocation = [15 10]; 
% endLocation = [90 50];
% path = findpath(prm,startLocation,endLocation) %find shortest path

% ====================END of PRM======================%

% ====================Starting Point of A*======================%

% validator = validatorOccupancyMap; % state validator object for collision check
% validator.Map = mapInflated; %assign map to the validator
% planner = plannerHybridAStar(validator,'MinTurningRadius',2,'MotionPrimitiveLength',2); %specify the costs
% startPose = [15 10 pi/2]; % [meters, meters, radians]
% goalPose = [90 50 -pi/2];
% refpath = plan(planner,startPose,goalPose);
% pathx=refpath.States
% path=pathx(:,1:2)

% ====================END of A*======================%

% ====================Starting Point of RRT======================%
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
sv.Map = mapInflated;
sv.ValidationDistance = 0.1;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
planner = plannerRRTStar(ss,sv);
planner.ContinueAfterGoalReached = true;
planner.MaxIterations = 5000;
planner.MaxConnectionDistance = 0.3;
start = [15, 10 0];
goal = [90, 50, 0];
rng(100, 'twister') % repeatable result
[pthObj, solnInfo] = plan(planner,start,goal);

pathx=pthObj.States
path=pathx(:,1:2)
% ====================END of RRT======================%


robotInitialLocation = path(1,:);
robotGoal = path(end,:);

initialOrientation = 0;

robotCurrentPose = [robotInitialLocation initialOrientation]';

%=========ROBOT KINEMATIC MODEL===========
robot = differentialDriveKinematics("TrackWidth", 5, "VehicleInputs", "VehicleSpeedHeadingRate", "WheelRadius" , 0.1);


%=============PATH VISUALIZATION===========
figure
show(mapInflated)
hold on
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); % tree expansion
hold on
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2); % draw path
hold on
plot(path(:,1), path(:,2),'k--d')
xlim([0 100])
ylim([0 80])
hold off

%=============PURE PURSUIT PARAMETERS===========
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 2;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.3;

%=============PURE PURSUIT PARAMETERS===========
goalRadius = 0.1; %Distance between robot origin to the target 
distanceToGoal = norm(robotInitialLocation - robotGoal); %Current Distance


% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;

myone_currentpose = 0;
figure
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
  
    % Update the plot
    hold off
    show(mapInflated)
    hold on
    %=======RRT MAP ==========
    plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); % tree expansion
    hold on
    plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2); % draw path
    %===END of RRT MAP==========
    
    %==== PRM MAP=====
%     show(prm)
    %==== END of PRM MAP =====
    
    %====A* MAP====
%     show(planner)
    %==== END of A* MAP===
    
    hold all
    % Plot path each instance so that it stays persistent while robot meshmoves
    plot(path(:,1), path(:,2),"k--d")
    title('DEU ROBOTICS')
    %hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0]; %Trajectory Vector
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]); %Rotation matrix
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 100])
    ylim([0 80])
    
    waitfor(vizRate); %Pause code execution to achieve desired execution rate
end

release(controller);
reset(vizRate);
