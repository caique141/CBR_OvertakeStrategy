%% Simulator
% Matlab script implements a CBR framework in autonomous racing

%% Define Initial Parameters
% Defines global parameters and generates map.

global nLaps

nLaps = 100;
CaseGenerator = false; % True if initial data set needs to be created
GPSAccuracy = false; % True to consider gps accuracy of 5 meters.
% Set learning threshold desired
LearningThreshold = 2;  % Distances between cases [m]

% Import track image
usedMap = generatemap();

% Import Case Base
% If Case Base does no exist, define CaseBase and badCaseBase as empty
CaseBase = [];
badCaseBase = [];

% Edit name for desired initial database
load('badCaseBase.mat')
load('CaseBase.mat')

% Set start and goal location for reference path generation
simulationParams = setParams();
trackSections = setSectors();

%% Define Reference Path
% Reference path can be using the path planner or
% using existing variables

% load('Poses.mat')
% load('waypoints.mat')
% load('distanceref.mat')

Pose_Opp = [];
n = length(trackSections);
waypoints = [];
Poses = [];
distanceref = [0];
for i=1:n
    if i == n
        startPoselocal = trackSections(i,:);
        goalPoselocal = trackSections(1,:);
        [path, Occmap] = Planner(startPoselocal,goalPoselocal,usedMap,...
            simulationParams,Pose_Opp,[],[]);
    else
        startPoselocal = trackSections(i,:);
        goalPoselocal = trackSections(i+1,:);
        [path, Occmap] = Planner(startPoselocal,goalPoselocal,usedMap,...
            simulationParams,Pose_Opp,[],[]);
    end
    newPoses = [path.States(:,1) path.States(:,2) path.States(:,3)];
    newwaypoints = [path.States(:,1) path.States(:,2)];

    Poses = [Poses; newPoses];
    waypoints = [waypoints; newwaypoints];
end


% Update poses and waypoints for total number of laps
totalPoses = repmat(Poses,nLaps,1);
totalWaypoints = totalPoses(:,1:2);

%% Define parameters and output variables

% Define output variables
Logs = [];
INFO = [];
newCases = [];
nCase = 0;
newCaseslaps = [];
retrievedList = [];
retrievedDist = [];
crashes.n = 0;
crashes.case = [];


% Simulation parameters
storeCase = 0;
i=0;
restart = 1;

% Start new figure to plot
figure


%% Start Simulation 
while true
    
    if restart
        % Run each time simulation is set to restart
        
        % Define Parameters for Car 1
        car1 = Car(1,30,10,simulationParams.LookAheadDist,totalWaypoints);
        tfin = [];
        car1finish = 0;
        poly1 = polyshape([-1.5 1.5 1.5 -1.5],[-0.9 -0.9 0.9 0.9]);
        
        % Define Parameters for Car 2
        car2 = Car(2,23,10,simulationParams.LookAheadDist,totalWaypoints);
        tfin2 = [];
        car2finish = 0;
        poly2 = polyshape([-1.5 1.5 1.5 -1.5],[-0.9 -0.9 0.9 0.9]);
        
        % Define Starting positions of cars
        % used random function
        if ~CaseGenerator
            startposeidx = randi([3 length(Poses)-2],1);
            startposeidx = ((car1.LapStats.currLap-1)*length(Poses))...
                +startposeidx;
            car1.Pose = totalPoses(startposeidx,:);
            car2.Pose = totalPoses(startposeidx+250,:);
            car1.TrackDistance = trackDistance(car1.Pose,...
                waypoints,distanceref);
            car2.TrackDistance = trackDistance(car2.Pose,...
                waypoints,distanceref);
            
        else
            % Random positioning for case generator
            startposeidx = randi([3 length(Poses)-20],1);
            
            car1.Pose = Poses(startposeidx,:);
            car1.TrackDistance = trackDistance(car1.Pose,...
                waypoints,distanceref);
            
            car2.Pose = Poses(startposeidx+17,:);
            car2.TrackDistance = trackDistance(car2.Pose...
                ,waypoints,distanceref);
            
            newCases = [];
            nCase = 0;
            newCaseslaps = [];
            retrievedList = [];
            retrievedDist = [];
        end
        
        % Restart Simulation variables
        stateCar1 = 1;
        stateCar2 = 1;
        car1ahead = [];
        storeCase = 0;
        storeBadCase = 0;
        i=0;
        restart = 0;
        overtakelap = nLaps;
        learncase = 0;
        
        % Save CaseBase if a new case is generated
        if ~isempty(CaseBase)
            save('CaseBasev5.mat','CaseBase');
            save('badCaseBasev5.mat','badCaseBase');
        end
        
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%    CAR 1 STATES    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if ~car1finish
        
        % Shift poses to define ego car as reference
        Distance1 = sqrt((car1.Pose(1) - Poses(:,1)).^2 + ...
            (car1.Pose(2) - Poses(:,2)).^2);
        [~,idx1]=min(abs(Distance1));
        shiftedposes = circshift(Poses,-(idx1-1));
                
        % Shift waypoints to define ego car as reference
        shiftedwaypoints = [shiftedposes(:,1) shiftedposes(:,2)];
        
        switch stateCar1
            case 1 % State 1 -> Follows reference path
                [car1,tfin] = car1.state1(i,tfin);
                
            case 2 % State 2 -> Follows reference path with opponent speed
                % Keep position behind opponent
                [car1,tfin] = car1.state2(i,tfin,car2.linearVelocity);
                
                % Logs information about section 1 of new case
                newCase.waypoints = [newCase.waypoints ; car1.Pose];
                newCase.speed = [newCase.speed ; car1.linearVelocity];
                
            case 3 % State 3 -> Uses Path generator to get new overtake
                % Overtaking zone needs to be defined in function
                % overtakeZone
                try
                    [car1,tfin] = car1.state3(i,tfin,32,car2,...
                        totalWaypoints,usedMap,simulationParams,...
                        shiftedposes,waypoints);
                    
                    % Logs information about section 2 of new case
                    newCase.waypoints = [newCase.waypoints ; car1.Pose];
                    newCase.speed = [newCase.speed ; car1.linearVelocity];
                    
                catch e
                    warning('Error path generaot. Restarting simulator.');
                    fprintf(1,'Error identifier:\n %s \n',e.identifier);
                    fprintf(1,'Error message:\n %s \n',e.message);
                    newCase.successful = 0;
                    storeBadCase = 1;
                    restart = 1;
                end
                
            case 4 % State 4 -> Follow case waypoints
                [car1,tfin] = car1.state4(i,tfin,retrievedCase);
                                    
                % Logs information about section 2 of new case
                newCase.waypoints = [newCase.waypoints ; car1.Pose];
                newCase.speed = [newCase.speed ; car1.linearVelocity];
        end
    end
    
    % Check if car1 completed all laps
    if car1.LapStats.currLap == (nLaps+1)
        car1finish = 1;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%    CAR 2 STATES    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if ~car2finish
        % State 1 -> Follows reference path
        % Opponent car will only follow refence path
        [car2,tfin2] = car2.state1(i,tfin2);
    end
    
    % Check if car2 completed all laps
    if car2.LapStats.currLap == (nLaps+1)
        car2finish = 1;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%  Transitions Car 1 %%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    if ~restart
        if ~car1finish
            switch stateCar1
                case 1 % Transition from State 1 to 2
                    distance2cars = distanceCars(car1,car2,waypoints);
                    if distance2cars < 8
                        
                        % Log transition information
                        stateCar1 = 2;
                        log.Transition = 'State 1 to 2';
                        log.Car1Pose = car1.Pose;
                        log.Car2Pose = car2.Pose;
                        log.Time = i;
                        Logs = [Logs; log];
                        
                        % Generate new Case
                        newCase = Case(length(CaseBase)+1,car1.Pose,...
                            distance2cars);
                        newCase.starttime = i;
                        newCase.successful = 1;
                        nCase = nCase + 1;
                        
                        if ~CaseGenerator
                            if GPSAccuracy
                                % Noise of Maximum 5m
                                noise= [rand*10-5 rand*10-5]; 
                            else
                                noise= [];
                            end
                            
                            % Case retrieval Phase
                            [retrievedCase,val] = retrieveCase...
                                (CaseBase,car1,waypoints,noise);
                            newCase.strategy = retrievedCase.strategy;
                            
                            % Log retrieved cases and distances
                            retrievedList = [retrievedList;...
                                retrievedCase.ID];
                            retrievedDist = [retrievedDist; val];
                            
                            % Check Learning Threshold
                            if val>LearningThreshold
                                learncase = 1;
                                newCase.prevCase = retrievedCase.ID;
                            end
                        end
                    end
                case 2 % Transition from State 2 to 3 or 4
                    if ~CaseGenerator
                        % Transition between case 2 and 4 
                        % Use case waypoints
                        % Set start of section 1 of current case
                        currentcase.egoPose = car1.Pose;                   
                        
                        Distance1 = sqrt((retrievedCase.startpose(1) - ...
                            shiftedwaypoints(:,1)).^2 + ...
                            (retrievedCase.startpose(2) - ...
                            shiftedwaypoints(:,2)).^2);
                        [val1,idx1]=min(abs(Distance1));
                        Distance2 = sqrt((currentcase.egoPose(1) - ...
                            shiftedwaypoints(:,1)).^2 + ...
                            (currentcase.egoPose(2) - ...
                            shiftedwaypoints(:,2)).^2);
                        [val2,idx2]=min(abs(Distance2));
                        
                        
                        aux = idx1-idx2;
                        if aux <= 1
                            stateCar1 = 4;
                            % Log transition between cases
                            log.Transition = 'State 2 to 4';
                            log.Car1Pose = car1.Pose;
                            log.Car2Pose = car2.Pose;
                            log.Time = i;
                            Logs = [Logs; log];
                            
                            % Define start of section 2 of current case
                            newCase.startpose = car1.Pose;
                        end
                    else
                        % Transition between case 2 and 3 
                        % Generate new path
                        % Check if car is in the overtake zone
                        % zone = overtakeZone(car1.Pose);
                        % if zone && stateCar1 == 2
                        if stateCar1 == 2
                            stateCar1 = 3;
                            % Log transition between cases
                            log.Transition = 'State 2 to 3';
                            log.Car1Pose = car1.Pose;
                            log.Car2Pose = car2.Pose;
                            log.Time = i;
                            Logs = [Logs; log];
                            
                            % Define start of section 2 of current case
                            newCase.startpose = car1.Pose;
                        end
                    end
                case 3 % Transition from State 3 to 1
                    
                    % Check if ego car is ahead of opponent
                    [car1ahead,idx] = checkCarAhead(totalWaypoints,...
                        car1,car2,shiftedwaypoints);
                    if idx>6 && car1ahead
                        car1.Controller.Waypoints = totalWaypoints;
                        car1.setLinearVelocity(30);
                        
                        % Log transition between cases
                        stateCar1 = 1;
                        log.Transition = 'State 3 to 1';
                        log.Car1Pose = car1.Pose;
                        log.Car2Pose = car2.Pose;
                        log.Time = i;
                        Logs = [Logs; log];
                        newCase.timereq = i-newCase.starttime;
                        newCases = [newCases; newCase];
                        storeCase = 1;
                        

                        Result.CaseBase = CaseBase;
                        Result.newCases = newCases;
                        Result.newCaseslaps = newCaseslaps;
                        Result.retrievedList = retrievedList;
                        Result.retrievedDist = retrievedDist;
                        Result.Log = Logs;
                        Result.Car1 = car1;
                        Result.Car2 = car2;
                        Result.i = i;
                        
                        disp('Overtake Completed')
                        overtakelap = car1.LapStats.currLap;
                    end
                case 4 % Transition from State 4 to 1
                    
                    % Check if ego car is ahead of opponent
                    Distance1 = sqrt((car1.Pose(1) - ...
                        retrievedCase.waypoints(:,1)).^2 + ...
                        (car1.Pose(2) - ...
                        retrievedCase.waypoints(:,2)).^2);
                    [~,idx1]=min(abs(Distance1));
                    if idx1 >= length(retrievedCase.waypoints)-2
                        
                        % Reset car speed and waypoints
                        car1.Controller.Waypoints = totalWaypoints;
                        car1.setLinearVelocity(30);
                        
                        % Log transition between cases
                        stateCar1 = 1;
                        log.Transition = 'State 4 to 1';
                        log.Car1Pose = car1.Pose;
                        log.Car2Pose = car2.Pose;
                        log.Time = i;
                        Logs = [Logs; log];

                        newCase.strategy = retrievedCase.strategy;
                        newCase.successful = 1;
                        newCase.timereq = i-newCase.starttime;
                        newCases = [newCases, newCase];
                        newCaseslaps = [newCaseslaps; ...
                            car1.LapStats.currLap];
                        
                        storeCase = learncase;
                        overtakelap = car1.LapStats.currLap;
                    end
            end
        end
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%  Crash Detection %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    
        polyplot1 = translate(rotate(poly1,car1.Pose(3)*180/pi),...
            [car1.Pose(1) car1.Pose(2)]);
        polyplot2 = translate(rotate(poly2,car2.Pose(3)*180/pi),...
            [car2.Pose(1) car2.Pose(2)]);
        crash = overlaps(polyplot1,polyplot2);
        if crash
            disp('Cars crashed - restart simulator');

            if ~isempty(newCase)
                % Set unsuccessful case
                newCase.successful = 0;
                storeBadCase = 1;
                newCase.timereq = i-newCase.starttime;
                crashes.case = [crashes.case; newCase];
            end
            crashes.n = crashes.n+1;
            % Restart Simulator
            restart = 1;
        end
        
        
    %%%%%%%%%%%%%%%%%%  Store Unsuccessful Case %%%%%%%%%%%%%%%%%%%%%%
        if storeBadCase
            if newCase.successful == 0
                badCaseBase = [badCaseBase,newCase];
            end
            newCase = [];
            restart = 1;
            storeBadCase = 0;
        end
        
    %%%%%%%%%%%%%%%%%%%%  Store Successful Case %%%%%%%%%%%%%%%%%%%%%%        
        if storeCase
            if newCase.successful == 1
                CaseBase = [CaseBase,newCase];
            end
            if CaseGenerator
                restart = 1;
            end
            storeCase = 0;
        end
    end
    
    
    % Create random repositioning of case after overtake lap completed
    if car1.LapStats.currLap == (overtakelap+1)
        if ~isempty(newCase)
            
            %Log results
            info.ID = newCase.ID;
            info.Case = newCase;
            info.lapTime = car1.LapStats.lapResults...
                (car1.LapStats.currLap-1);
            INFO = [INFO info];
            
            Result.CaseBase = CaseBase;
            Result.newCases = newCases;
            Result.newCaseslaps = newCaseslaps;
            Result.retrievedList = retrievedList;
            Result.retrievedDist = retrievedDist;
            Result.Log = Logs;
            Result.Car1 = car1;
            Result.Car2 = car2;
            Result.i = i;
            Result.info = INFO;
        end
        restart = 1;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%  End of race %%%%%%%%%%%%%%%%%%%%%%%%%  
    if car1finish && car2finish
        disp('------------End of Race-----------------------');
        Result.CaseBase = CaseBase;
        Result.newCases = newCases;
        Result.newCaseslaps = newCaseslaps;
        Result.retrievedList = retrievedList;
        Result.retrievedDist = retrievedDist;
        Result.Log = Logs;
        Result.Car1 = car1;
        Result.Car2 = car2;
        Result.i = i;
        name = strcat ('Result_Race',datestr(now,'dd_mm_HH_MM'));
        save(name,'Result');
        break
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%  Plot %%%%%%%%%%%%%%%%%%%%%%%%%  
    if rem(i,0.05) <= 0.01
        plot(Occmap,'Inflation','off');
        hold on
        plot(polyplot1);
        plot(polyplot2);
        drawnow
        hold off
    end
    i= i + 0.01;
end


%% FUNCTIONS

function distance = trackDistance(car1Pose,waypoints,distanceref) 
%TRACKDISTANCE Calculates track distances based on reference path
%   Calculates the current track distance of the car from the start line
        Distance1 = sqrt((car1Pose(1) - waypoints(:,1)).^2 + ...
            (car1Pose(2) - waypoints(:,2)).^2);
        [~,idx]=min(abs(Distance1));
        distance = distanceref(idx);
end

function sectors = setSectors()
%SETSECTORS Defines referece points on track
%   Reference points are used by planner to generate reference path.
%   Must be eddited if different track is used.

sectors = [[9,60,pi/2];...
    [40,105,0];...
    [104,27,-pi/2];...
    [44,32,5*pi/4]];
end

function params = setParams()
%SETPARAMS Define main simulation parameters
%   Define the parameters from the controller and vehicle model

params.DistanceFromGoal = 0.5;

% Pure Pursuit parameters
params.DesiredLinVel = 3; % [m/s]
params.MaximumAngVel = 10.0; % [rad/s]
params.LookAheadDist = 5; % [m]

%   Bicycle Model Parameters
% Wheelbase = 0.25; % [m]
% MaximumSteeringAngle = sA; % [rad]
% InitialState = 0; % Pose [x y ang]

params.minimumRadius = 10.5; % [m]
R = params.minimumRadius;
l = 2.8; % [m]
params.wheelBase = l; % [m]
a2 = l/2;
%   Max steering angle
sA = atan(1/(sqrt((R^2 - a2^2)/l^2)));
params.steeringAngle = sA;
% Motion Primitive Length
circ = 2*pi*R;
params.motionPrimitive = circ/5;
end

function zone = overtakeZone(EgoPose)

if ~isempty(EgoPose) 
    
    xq = EgoPose(1);
    yq = EgoPose(2);
    
% Straight 0 
    xv = [0 17 17 0 0];
    yv = [20 20 90 90 20];
    
    [in1,on] = inpolygon(xq,yq,xv,yv);
    
% Straight 1
xv = [25 89 89 25 25];
yv = [97 97 111 111 97];

[in2,on] = inpolygon(xq,yq,xv,yv);

% Straight 2
    xv = [95 112 112 95 95];
    yv = [24 24 92 92 24];
    
    [in3,on] = inpolygon(xq,yq,xv,yv);

% Straight 3
    xv = [53.25 67.25 18.25 27.25 53.25];
    yv = [55.25 45.25 20.25 4.75 55.25];

    [in4,on] = inpolygon(xq,yq,xv,yv); 
    
    sum = in1+in2+in3+in4;
    zone = sum;
else
    zone = false;
end

end

function map = generatemap()
%GENERATEMAP Track image import and treatment
%   Imports the track image as bmp, resize and converts into single format.
%   Must be adjusted for different tracks.

% Generate logical map 225x225
Track3 = im2bw(imread('track3.bmp'),0.5);

% Converts from logical to single format
combinedMap = im2single(Track3);

map = combinedMap;
end

function distance2cars = distanceCars(car1,car2,waypoints)
%DISTANCECARS Check the fisical distance between cars
%   Used the reference waypoints to calculate the physical distances
%   between the two given cars.

%   Checks in which position of the reference waypoints the car1 is
%   located.
Distance1 = sqrt((car1.Pose(1) - waypoints(:,1)).^2 + ...
    (car1.Pose(2) - waypoints(:,2)).^2);
[~,idx1]=min(abs(Distance1));

shiftedwaypoints = circshift(waypoints,-(idx1-1));

shifteddistance = NaN(length(shiftedwaypoints),1);
shifteddistance(1) = 0;
for i=2:length(shiftedwaypoints)
    shifteddistance(i) = (shifteddistance(i-1) + ...
        pdist([shiftedwaypoints(i,1) shiftedwaypoints(i,2);...
        shiftedwaypoints(i-1,1) shiftedwaypoints(i-1,2)]));
end

shiftDistance2 = sqrt((car2.Pose(1) - shiftedwaypoints(:,1)).^2 + ...
    (car2.Pose(2) - shiftedwaypoints(:,2)).^2);
[~,idx2s]=min(abs(shiftDistance2));

distance2cars = shifteddistance(idx2s);
end

        
      