classdef mobilerobot_simulation_APP < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                      matlab.ui.Figure
        Image2                        matlab.ui.control.Image
        Image                         matlab.ui.control.Image
        DEUROBOTICSLabel              matlab.ui.control.Label
        UITable                       matlab.ui.control.Table
        ReadyPathButton               matlab.ui.control.Button
        RRTButton                     matlab.ui.control.Button
        AButton                       matlab.ui.control.Button
        PRMButton                     matlab.ui.control.Button
        TextArea                      matlab.ui.control.TextArea
        InitalPositionXAxisEditFieldLabel  matlab.ui.control.Label
        InitalPositionXAxisEditField  matlab.ui.control.NumericEditField
        InitalPositionYAxisEditFieldLabel  matlab.ui.control.Label
        InitalPositionYAxisEditField  matlab.ui.control.NumericEditField
        GoalPositionXAxisEditFieldLabel  matlab.ui.control.Label
        GoalPositionXAxisEditField    matlab.ui.control.NumericEditField
        GoalPositionYAxisEditFieldLabel  matlab.ui.control.Label
        GoalPositionYAxisEditField    matlab.ui.control.NumericEditField
        ObstacleDetectionLampLabel    matlab.ui.control.Label
        ObstacleDetectionLamp         matlab.ui.control.Lamp
        InitialOrientationEditFieldLabel  matlab.ui.control.Label
        InitialOrientationEditField   matlab.ui.control.NumericEditField
        ObstacleXEditFieldLabel       matlab.ui.control.Label
        ObstacleXEditField            matlab.ui.control.NumericEditField
        NewXEditFieldLabel            matlab.ui.control.Label
        NewXEditField                 matlab.ui.control.NumericEditField
        NewYEditFieldLabel            matlab.ui.control.Label
        NewYEditField                 matlab.ui.control.NumericEditField
        ObstacleYEditFieldLabel       matlab.ui.control.Label
        ObstacleYEditField            matlab.ui.control.NumericEditField
        User1Button                   matlab.ui.control.Button
        User2Button                   matlab.ui.control.Button
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: ReadyPathButton
        function ReadyPathButtonPushed(app, event)
            data = readtable("mypath.xlsx","Sheet",1);
            app.UITable.Data = data;
            data.Properties.VariableNames{1} = 'X Position';
            data.Properties.VariableNames{2} = 'Y Position';
            app.UITable.ColumnName = data.Properties.VariableNames;
            
            x_pos = table2array(data(:,"X Position"));
            y_pos = table2array(data(:,"Y Position"));
            path = [x_pos y_pos];
            
            %%%%%%%%%%%%%%%%%%%%%%%
            map = binaryOccupancyMap(100, 80, 1);
            occ = zeros(80, 100);
            
            occ(1,:) = 1; %Top
            occ(end,:) = 1; %Buttom
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
            
            flag = 0;
 
            for i = 1:length(path)-1
                if occ(path(i,2),path(i,1))==1

                    app.ObstacleYEditField.Value = path(i,2);
                    app.ObstacleXEditField.Value = path(i,1);
                    app.NewXEditField.Value = path(i,1)-5;
                    app.NewYEditField.Value = path(i,2)-5;
                    path(i,1) = path(i,1)-5;
                    path(i,2) = path(i,2)-5;
                    flag =1;
                end
            end
            
            if flag ==1
                app.ObstacleDetectionLamp.Color = [0.39,0.83,0.07];
                app.ObstacleDetectionLampLabel.Text = 'Obstacle Detected Path Changed'
            end
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
            goalRadius = 0.1; 
            distanceToGoal = norm(robotInitialLocation - robotGoal); 
            
            
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
                
                % Update the current pose
                
                if occ(floor(robotCurrentPose(2)),floor(robotCurrentPose(1)))== 1
                    myone_currentpose = robotCurrentPose
                end
                robotCurrentPose = robotCurrentPose + vel*sampleTime; 
                
            %     if occ(floor(robotCurrentPose(1)),floor(robotCurrentPose(2))) == 1
            %         robotCurrentPose(1) = robotCurrentPose(1)-1;
            %         robotCurrentPose(2) = robotCurrentPose(2)-1;
            %     end
                
                % Re-compute the distance to the goal
                distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
                
              
                % Update the plot
                hold off
                show(mapInflated)
                hold all
                % Plot path each instance so that it stays persistent while robot mesh
                % moves
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

        end

        % Button pushed function: RRTButton
        function RRTButtonPushed(app, event)
            map = binaryOccupancyMap(100, 80, 1);
            occ = zeros(80, 100);
            
            occ(1,:) = 1; 
            occ(end,:) = 1; 
            occ([1:30, 51:80],1) = 1; 
            occ([1:30, 51:80],end) = 1;
            
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
            
            %%%%%%%%%%%% PATH FINDER WITH RRT %%%%%%%%%%%%%
            ss = stateSpaceSE2;
            sv = validatorOccupancyMap(ss);
            sv.Map = mapInflated;
            sv.ValidationDistance = 0.1;
            ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
            planner = plannerRRTStar(ss,sv);
            planner.ContinueAfterGoalReached = true;
            planner.MaxIterations = 5000;
            planner.MaxConnectionDistance = 0.3;
            
            initial_x = app.InitalPositionXAxisEditField.Value;
            initial_y = app.InitalPositionYAxisEditField.Value;
            initial_ori = app.InitialOrientationEditField.Value;
           
            start = [initial_x, initial_y, initial_ori];
            
            goal_x = app.GoalPositionXAxisEditField.Value;
            goal_y = app.GoalPositionYAxisEditField.Value;
            
            goal = [goal_x, goal_y, 0];
            rng(100, 'twister') % repeatable result
            [pthObj, solnInfo] = plan(planner,start,goal);
            
            pathx=pthObj.States
            path=pathx(:,1:2)
            
            app.UITable.Data = path;
            data.Properties.VariableNames{1} = 'X Position';
            data.Properties.VariableNames{2} = 'Y Position';
            app.UITable.ColumnName = data.Properties.VariableNames;
            
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
            hold off
            
            %=============PURE PURSUIT PARAMETERS===========
            controller = controllerPurePursuit;
            controller.Waypoints = path;
            controller.DesiredLinearVelocity = 2;
            controller.MaxAngularVelocity = 2;
            controller.LookaheadDistance = 0.3;
            
            %=============PURE PURSUIT PARAMETERS===========
            goalRadius = 0.1; 
            distanceToGoal = norm(robotInitialLocation - robotGoal); 
            
            
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
                
                % Update the current pose
                
                if occ(floor(robotCurrentPose(2)),floor(robotCurrentPose(1)))== 1
                    myone_currentpose = robotCurrentPose
                end
                robotCurrentPose = robotCurrentPose + vel*sampleTime; 
                
            %     if occ(floor(robotCurrentPose(1)),floor(robotCurrentPose(2))) == 1
            %         robotCurrentPose(1) = robotCurrentPose(1)-1;
            %         robotCurrentPose(2) = robotCurrentPose(2)-1;
            %     end
                
                % Re-compute the distance to the goal
                distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
                
              
                % Update the plot
                hold off
                show(mapInflated)
                hold on
                plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); % tree expansion
                hold on
                plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2); % draw path
                hold all
                % Plot path each instance so that it stays persistent while robot mesh
                % moves
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
        end

        % Button pushed function: AButton
        function AButtonPushed(app, event)
            map = binaryOccupancyMap(100, 80, 1);
            occ = zeros(80, 100);
            
            occ(1,:) = 1;
            occ(end,:) = 1; 
            occ([1:30, 51:80],1) = 1; 
            occ([1:30, 51:80],end) = 1;
            
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
            
            %%%%%%%%%%%% PATH FINDER WITH Astar %%%%%%%%%%%%%
            validator = validatorOccupancyMap; % state validator object for collision check
            validator.Map = mapInflated; %assign map to the validator
            planner = plannerHybridAStar(validator,'MinTurningRadius',2,'MotionPrimitiveLength',2); %specify the costs
            
            initial_x = app.InitalPositionXAxisEditField.Value;
            initial_y = app.InitalPositionYAxisEditField.Value;
            initial_ori = app.InitialOrientationEditField.Value;
           
            goal_x = app.GoalPositionXAxisEditField.Value;
            goal_y = app.GoalPositionYAxisEditField.Value;
            
            startPose = [initial_x initial_y initial_ori]; % [meters, meters, radians]
            goalPose = [goal_x goal_y -pi/2];
            refpath = plan(planner,startPose,goalPose);
            pathx=refpath.States
            path=pathx(:,1:2)
            
            app.UITable.Data = path;
            data.Properties.VariableNames{1} = 'X Position';
            data.Properties.VariableNames{2} = 'Y Position';
            app.UITable.ColumnName = data.Properties.VariableNames;
            
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
            show(planner)
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
            goalRadius = 0.1; 
            distanceToGoal = norm(robotInitialLocation - robotGoal); 
            
            
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
                
                % Update the current pose
                
                if occ(floor(robotCurrentPose(2)),floor(robotCurrentPose(1)))== 1
                    myone_currentpose = robotCurrentPose
                end
                robotCurrentPose = robotCurrentPose + vel*sampleTime; 
                
            %     if occ(floor(robotCurrentPose(1)),floor(robotCurrentPose(2))) == 1
            %         robotCurrentPose(1) = robotCurrentPose(1)-1;
            %         robotCurrentPose(2) = robotCurrentPose(2)-1;
            %     end
                
                % Re-compute the distance to the goal
                distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
                
              
                % Update the plot
                hold off
                show(mapInflated)
                hold on
                show(planner)
                hold all
                % Plot path each instance so that it stays persistent while robot mesh
                % moves
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
        end

        % Button pushed function: PRMButton
        function PRMButtonPushed(app, event)
            map = binaryOccupancyMap(100, 80, 1);
            occ = zeros(80, 100);
            
            occ(1,:) = 1; 
            occ(end,:) = 1; 
            occ([1:30, 51:80],1) = 1;
            occ([1:30, 51:80],end) = 1;
            
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
            
            %%%%%%%%%%%% PATH FINDER WITH RRT %%%%%%%%%%%%%
            
            initial_x = app.InitalPositionXAxisEditField.Value;
            initial_y = app.InitalPositionYAxisEditField.Value;
            initial_ori = app.InitialOrientationEditField.Value;
           
            goal_x = app.GoalPositionXAxisEditField.Value;
            goal_y = app.GoalPositionYAxisEditField.Value;
            
            prm = mobileRobotPRM(mapInflated,60); % assign 60 nodes on map
            startLocation = [initial_x initial_y]; 
            endLocation = [goal_x goal_y];
            path = findpath(prm,startLocation,endLocation) %find shortest path
            
            app.UITable.Data = path;
            data.Properties.VariableNames{1} = 'X Position';
            data.Properties.VariableNames{2} = 'Y Position';
            app.UITable.ColumnName = data.Properties.VariableNames;
            
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
            show(prm)
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
            goalRadius = 0.1; 
            distanceToGoal = norm(robotInitialLocation - robotGoal);
            
            
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
                
                % Update the current pose
                
                if occ(floor(robotCurrentPose(2)),floor(robotCurrentPose(1)))== 1
                    myone_currentpose = robotCurrentPose
                end
                robotCurrentPose = robotCurrentPose + vel*sampleTime; 
                
            %     if occ(floor(robotCurrentPose(1)),floor(robotCurrentPose(2))) == 1
            %         robotCurrentPose(1) = robotCurrentPose(1)-1;
            %         robotCurrentPose(2) = robotCurrentPose(2)-1;
            %     end
                
                % Re-compute the distance to the goal
                distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
                
              
                % Update the plot
                hold off
                show(mapInflated)
                hold on
                show(prm)
                hold all
                % Plot path each instance so that it stays persistent while robot mesh
                % moves
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
        end

        % Button pushed function: User1Button
        function User1ButtonPushed(app, event)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%                                            %%%%
            %%%%                                            %%%%
            %%%%    PLEASE WRITE NEW ALGORTIHM HERE         %%%%
            %%%%                                            %%%%
            %%%%                                            %%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end

        % Button pushed function: User2Button
        function User2ButtonPushed(app, event)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%                                            %%%%
            %%%%                                            %%%%
            %%%%    PLEASE WRITE NEW ALGORTIHM HERE         %%%%
            %%%%                                            %%%%
            %%%%                                            %%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 640 480];
            app.UIFigure.Name = 'MATLAB App';

            % Create Image2
            app.Image2 = uiimage(app.UIFigure);
            app.Image2.Position = [1 370 117 111];
            app.Image2.ImageSource = 'Deu.jpg';

            % Create Image
            app.Image = uiimage(app.UIFigure);
            app.Image.Position = [525 370 116 111];
            app.Image.ImageSource = 'Muh.jpg';

            % Create DEUROBOTICSLabel
            app.DEUROBOTICSLabel = uilabel(app.UIFigure);
            app.DEUROBOTICSLabel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.DEUROBOTICSLabel.FontSize = 25;
            app.DEUROBOTICSLabel.Position = [223 416 196 30];
            app.DEUROBOTICSLabel.Text = 'DEU ROBOTICS';

            % Create UITable
            app.UITable = uitable(app.UIFigure);
            app.UITable.ColumnName = {'Column 1'; 'Column 2'};
            app.UITable.RowName = {};
            app.UITable.Position = [1 149 302 185];

            % Create ReadyPathButton
            app.ReadyPathButton = uibutton(app.UIFigure, 'push');
            app.ReadyPathButton.ButtonPushedFcn = createCallbackFcn(app, @ReadyPathButtonPushed, true);
            app.ReadyPathButton.Position = [18 121 100 22];
            app.ReadyPathButton.Text = 'ReadyPath';

            % Create RRTButton
            app.RRTButton = uibutton(app.UIFigure, 'push');
            app.RRTButton.ButtonPushedFcn = createCallbackFcn(app, @RRTButtonPushed, true);
            app.RRTButton.Position = [169 121 100 22];
            app.RRTButton.Text = 'RRT';

            % Create AButton
            app.AButton = uibutton(app.UIFigure, 'push');
            app.AButton.ButtonPushedFcn = createCallbackFcn(app, @AButtonPushed, true);
            app.AButton.Position = [169 85 100 22];
            app.AButton.Text = 'A*';

            % Create PRMButton
            app.PRMButton = uibutton(app.UIFigure, 'push');
            app.PRMButton.ButtonPushedFcn = createCallbackFcn(app, @PRMButtonPushed, true);
            app.PRMButton.Position = [169 47 100 22];
            app.PRMButton.Text = 'PRM';

            % Create TextArea
            app.TextArea = uitextarea(app.UIFigure);
            app.TextArea.HorizontalAlignment = 'center';
            app.TextArea.Position = [1 1 185 19];
            app.TextArea.Value = {'Kutay GÖNEN'};

            % Create InitalPositionXAxisEditFieldLabel
            app.InitalPositionXAxisEditFieldLabel = uilabel(app.UIFigure);
            app.InitalPositionXAxisEditFieldLabel.HorizontalAlignment = 'right';
            app.InitalPositionXAxisEditFieldLabel.Position = [398 169 122 22];
            app.InitalPositionXAxisEditFieldLabel.Text = 'Inital Position - X Axis';

            % Create InitalPositionXAxisEditField
            app.InitalPositionXAxisEditField = uieditfield(app.UIFigure, 'numeric');
            app.InitalPositionXAxisEditField.Position = [535 169 100 22];

            % Create InitalPositionYAxisEditFieldLabel
            app.InitalPositionYAxisEditFieldLabel = uilabel(app.UIFigure);
            app.InitalPositionYAxisEditFieldLabel.HorizontalAlignment = 'right';
            app.InitalPositionYAxisEditFieldLabel.Position = [398 139 122 22];
            app.InitalPositionYAxisEditFieldLabel.Text = 'Inital Position - Y Axis';

            % Create InitalPositionYAxisEditField
            app.InitalPositionYAxisEditField = uieditfield(app.UIFigure, 'numeric');
            app.InitalPositionYAxisEditField.Position = [535 139 100 22];

            % Create GoalPositionXAxisEditFieldLabel
            app.GoalPositionXAxisEditFieldLabel = uilabel(app.UIFigure);
            app.GoalPositionXAxisEditFieldLabel.HorizontalAlignment = 'right';
            app.GoalPositionXAxisEditFieldLabel.Position = [398 63 122 22];
            app.GoalPositionXAxisEditFieldLabel.Text = 'Goal Position - X Axis';

            % Create GoalPositionXAxisEditField
            app.GoalPositionXAxisEditField = uieditfield(app.UIFigure, 'numeric');
            app.GoalPositionXAxisEditField.Position = [535 63 100 22];

            % Create GoalPositionYAxisEditFieldLabel
            app.GoalPositionYAxisEditFieldLabel = uilabel(app.UIFigure);
            app.GoalPositionYAxisEditFieldLabel.HorizontalAlignment = 'right';
            app.GoalPositionYAxisEditFieldLabel.Position = [398 27 122 22];
            app.GoalPositionYAxisEditFieldLabel.Text = 'Goal Position - Y Axis';

            % Create GoalPositionYAxisEditField
            app.GoalPositionYAxisEditField = uieditfield(app.UIFigure, 'numeric');
            app.GoalPositionYAxisEditField.Position = [535 27 100 22];

            % Create ObstacleDetectionLampLabel
            app.ObstacleDetectionLampLabel = uilabel(app.UIFigure);
            app.ObstacleDetectionLampLabel.HorizontalAlignment = 'right';
            app.ObstacleDetectionLampLabel.Position = [360 332 223 22];
            app.ObstacleDetectionLampLabel.Text = 'Obstacle Detection';

            % Create ObstacleDetectionLamp
            app.ObstacleDetectionLamp = uilamp(app.UIFigure);
            app.ObstacleDetectionLamp.Position = [598 333 20 20];
            app.ObstacleDetectionLamp.Color = [0.502 0.502 0.502];

            % Create InitialOrientationEditFieldLabel
            app.InitialOrientationEditFieldLabel = uilabel(app.UIFigure);
            app.InitialOrientationEditFieldLabel.HorizontalAlignment = 'right';
            app.InitialOrientationEditFieldLabel.Position = [425 100 95 22];
            app.InitialOrientationEditFieldLabel.Text = 'Initial Orientation';

            % Create InitialOrientationEditField
            app.InitialOrientationEditField = uieditfield(app.UIFigure, 'numeric');
            app.InitialOrientationEditField.Position = [535 100 100 22];

            % Create ObstacleXEditFieldLabel
            app.ObstacleXEditFieldLabel = uilabel(app.UIFigure);
            app.ObstacleXEditFieldLabel.HorizontalAlignment = 'right';
            app.ObstacleXEditFieldLabel.Position = [384 300 64 22];
            app.ObstacleXEditFieldLabel.Text = 'Obstacle X';

            % Create ObstacleXEditField
            app.ObstacleXEditField = uieditfield(app.UIFigure, 'numeric');
            app.ObstacleXEditField.Position = [463 300 37 22];

            % Create NewXEditFieldLabel
            app.NewXEditFieldLabel = uilabel(app.UIFigure);
            app.NewXEditFieldLabel.HorizontalAlignment = 'right';
            app.NewXEditFieldLabel.Position = [401 267 47 22];
            app.NewXEditFieldLabel.Text = 'New X';

            % Create NewXEditField
            app.NewXEditField = uieditfield(app.UIFigure, 'numeric');
            app.NewXEditField.Position = [463 267 37 22];

            % Create NewYEditFieldLabel
            app.NewYEditFieldLabel = uilabel(app.UIFigure);
            app.NewYEditFieldLabel.HorizontalAlignment = 'right';
            app.NewYEditFieldLabel.Position = [534 267 47 22];
            app.NewYEditFieldLabel.Text = 'New Y';

            % Create NewYEditField
            app.NewYEditField = uieditfield(app.UIFigure, 'numeric');
            app.NewYEditField.Position = [596 267 37 22];

            % Create ObstacleYEditFieldLabel
            app.ObstacleYEditFieldLabel = uilabel(app.UIFigure);
            app.ObstacleYEditFieldLabel.HorizontalAlignment = 'right';
            app.ObstacleYEditFieldLabel.Position = [519 300 64 22];
            app.ObstacleYEditFieldLabel.Text = 'Obstacle Y';

            % Create ObstacleYEditField
            app.ObstacleYEditField = uieditfield(app.UIFigure, 'numeric');
            app.ObstacleYEditField.Position = [598 300 37 22];

            % Create User1Button
            app.User1Button = uibutton(app.UIFigure, 'push');
            app.User1Button.ButtonPushedFcn = createCallbackFcn(app, @User1ButtonPushed, true);
            app.User1Button.Position = [18 84 100 22];
            app.User1Button.Text = 'User1';

            % Create User2Button
            app.User2Button = uibutton(app.UIFigure, 'push');
            app.User2Button.ButtonPushedFcn = createCallbackFcn(app, @User2ButtonPushed, true);
            app.User2Button.Position = [18 47 100 22];
            app.User2Button.Text = 'User2';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = mobilerobot_simulation_APP

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end