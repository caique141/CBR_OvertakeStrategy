classdef Car
    %CAR Class representing the cars used in the simulator
    %   This class defines the attributes, methods and states of each car
    %   used in the simulator

    properties
        ID
        linearVelocity
        angularVelocity
        Model
        Controller
        Pose
        Trajectory
        TrackDistance
        LapStats
    end
    
    methods
        function car = Car(ID,linearVelocity, angularVelocity,...
                LookAheadDist,refWaypoints)
            %CAR Constructor function of class
            %   Defines main parameters required by the car class
            global nLaps
            if nargin > 0
                car.ID = ID;
                car.Trajectory = [];
                car.linearVelocity = linearVelocity;
                car.angularVelocity = angularVelocity;
                car.Model = bicycleKinematics;
                car.Controller = controllerPurePursuit...
                    ('DesiredLinearVelocity',linearVelocity,...
                    'LookaheadDistance', LookAheadDist,...
                    'MaxAngularVelocity',angularVelocity,...
                    'Waypoints',refWaypoints);
                car.LapStats.prevlapFlag = 0;
                car.LapStats.lapFlag = 0;
                car.LapStats.lapTimes = zeros(nLaps,1);
                car.LapStats.lapResults = zeros(nLaps,1);
                car.LapStats.currLap = 1;
            end
        end
        
        function data = simstep(obj,i)
            %SIMSTEP Implements main simulation step
            %   Controller computes linear and angular velocity required 
            %   by the vehicle.
            %   Car model recieves commands as inputs and gives output in 
            %   a defined time step.
            
            [vel,angvel] = obj.Controller(obj.Pose);
            [t,y] = ode45(@(t,y)derivative(obj.Model,y,[vel angvel]),...
                [(i) (i+0.01)],obj.Pose);
            data = [t,y];
        end
        
        function obj = setLinearVelocity(obj,linvel)
            %SETLINEARVELOCITY Set a new linear velocity for the vehicle
            %   linear velocity is set in the required attributes of the
            %   vehicle.
            
            obj.linearVelocity = linvel;
            obj.Controller.DesiredLinearVelocity = linvel;
        end
        
        function [obj,tfin] = state1(obj,i,tfin)
            %STATE1 Car reasoning state 1
            %   Defines functions to follow reference path
            
            stepresult = obj.simstep(i);
            % stepresult = [t y]; t -> time, y -> pose [x y rad]
            obj.TrackDistance = [obj.TrackDistance;...
                (obj.TrackDistance(end) + ...
                pdist([obj.Pose(1) obj.Pose(2); ...
                stepresult(end,2) stepresult(end,3)]))];
            obj.Pose = [stepresult(end,2) stepresult(end,3) ...
                stepresult(end,4)];
            obj.Trajectory = [obj.Trajectory; obj.Pose];
            
            tfin = [tfin;stepresult(end,1)];
            
            % Check if lap is completed
            obj.LapStats.prevlapFlag = obj.LapStats.lapFlag;
            trackArea = [0 60;17 60;17 62;0 62; 0 60];
            [in,~] = inpolygon(obj.Pose(1),obj.Pose(2),...
                trackArea(:,1),trackArea(:,2));
            if in
                obj.LapStats.lapFlag = 1;
            else
                obj.LapStats.lapFlag =0;
            end
            % Update lap stats
            if obj.LapStats.lapFlag == 1 && obj.LapStats.prevlapFlag ==0
                if obj.LapStats.currLap > 0
                    obj.LapStats.lapTimes(obj.LapStats.currLap) = ...
                        stepresult(end,1);
                end
                if obj.LapStats.currLap == 1
                    obj.LapStats.lapResults(obj.LapStats.currLap) = ...
                        stepresult(end,1);
                elseif obj.LapStats.currLap > 1
                    obj.LapStats.lapResults(obj.LapStats.currLap) = ...
                        stepresult(end,1) - ...
                        obj.LapStats.lapTimes(obj.LapStats.currLap-1);
                end
                obj.LapStats.currLap = obj.LapStats.currLap+1;
            end
        end
        
        function [obj,tfin] = state2(obj,i,tfin,linvel)
            %STATE2 Car reasoning state 2
            %   Defines functions to follow opponent
            
            obj = obj.setLinearVelocity(linvel);
            stepresult = obj.simstep(i);
            % stepresult = [t y]; t -> time, y -> pose [x y rad]
            obj.TrackDistance = [obj.TrackDistance;...
                (obj.TrackDistance(end) + ...
                pdist([obj.Pose(1) obj.Pose(2); ...
                stepresult(end,2) stepresult(end,3)]))];
            obj.Pose = [stepresult(end,2) stepresult(end,3) ...
                stepresult(end,4)];
            obj.Trajectory = [obj.Trajectory; obj.Pose];
            
            tfin = [tfin;stepresult(end,1)];
            
            % Check if lap is completed
            obj.LapStats.prevlapFlag = obj.LapStats.lapFlag;
            trackArea = [0 60;17 60;17 62;0 62; 0 60];
            [in,~] = inpolygon(obj.Pose(1),obj.Pose(2),trackArea(:,1),...
                trackArea(:,2));
            if in
                obj.LapStats.lapFlag = 1;
            else
                obj.LapStats.lapFlag =0;
            end
            % Update lap stats
            if obj.LapStats.lapFlag == 1 && obj.LapStats.prevlapFlag ==0
                if obj.LapStats.currLap > 0
                    obj.LapStats.lapTimes(obj.LapStats.currLap) = ...
                        stepresult(end,1);
                end
                if obj.LapStats.currLap == 1
                    obj.LapStats.lapResults(obj.LapStats.currLap) = ...
                        stepresult(end,1);
                elseif obj.LapStats.currLap > 1
                    obj.LapStats.lapResults(obj.LapStats.currLap) = ...
                        stepresult(end,1) - ...
                        obj.LapStats.lapTimes(obj.LapStats.currLap-1);
                end
                obj.LapStats.currLap = obj.LapStats.currLap+1;
            end
        end
        
        function [obj,tfin] = state3(obj,i,tfin,linvel,car2,...
                totalWaypoints,usedMap,simulationParams,...
                shiftedposes,waypoints)
            %STATE3 Car reasoning state 3
            %   Defines functions to generate new overtake
            obj = obj.setLinearVelocity(linvel);
            if rem(i,0.02) < 0.01
                startPoselocal = obj.Pose;
                goalPoselocal=shiftedposes(50,:);
                Pose_Opp = car2.Pose;
                nowaypoints = 0;
                [car1ahead,~] = checkCarAhead(totalWaypoints,obj,...
                    car2,shiftedposes);
                % Search for new path
                [path, ~] = Planner(startPoselocal,goalPoselocal,...
                    usedMap,simulationParams,Pose_Opp,car1ahead,waypoints);
                localwaypoints = [path.States(:,1) path.States(:,2)];
                while isempty(localwaypoints)
                    if simulationParams.minimumRadius > 4
                        simulationParams.minimumRadius = ...
                            simulationParams.minimumRadius - 1;
                        simulationParams.steeringAngle = ...
                            atan(1/...
                            (sqrt(((simulationParams.minimumRadius)^2 - ...
                            (simulationParams.wheelBase/2)^2)/...
                            simulationParams.wheelBase^2)));
                        simulationParams.motionPrimitive = ...
                            (2*pi*simulationParams.minimumRadius)/5;
                        [path, ~] = Planner(startPoselocal,...
                            goalPoselocal,usedMap,simulationParams,...
                            Pose_Opp,car1ahead,waypoints);
                        localwaypoints = [path.States(:,1) ...
                            path.States(:,2)];
                    else
                        nowaypoints = 1;
                        break
                    end
                end
                if nowaypoints
                    obj = obj.setLinearVelocity(car2.linearVelocity);
                    obj.Controller.Waypoints = totalWaypoints;
                else
                    obj.Controller.Waypoints = localwaypoints;
                end
                simulationParams.minimumRadius = 10.5;
                simulationParams.steeringAngle = ...
                    atan(1/(sqrt(((simulationParams.minimumRadius)^2 - ...
                    (simulationParams.wheelBase/2)^2)/...
                    simulationParams.wheelBase^2)));
                simulationParams.motionPrimitive = ...
                    (2*pi*simulationParams.minimumRadius)/5;
            end
            
            stepresult = obj.simstep(i);
            % stepresult = [t y]; t -> time, y -> pose [x y rad]
            obj.TrackDistance = [obj.TrackDistance;...
                (obj.TrackDistance(end) + ...
                pdist([obj.Pose(1) obj.Pose(2); ...
                stepresult(end,2) stepresult(end,3)]))];
            obj.Pose = [stepresult(end,2) stepresult(end,3) ...
                stepresult(end,4)];
            obj.Trajectory = [obj.Trajectory; obj.Pose];
            
            tfin = [tfin;stepresult(end,1)];
            
            % Check if lap is completed
            obj.LapStats.prevlapFlag = obj.LapStats.lapFlag;
            trackArea = [0 60;17 60;17 62;0 62; 0 60];
            [in,~] = inpolygon(obj.Pose(1),obj.Pose(2),...
                trackArea(:,1),trackArea(:,2));
            if in
                obj.LapStats.lapFlag = 1;
            else
                obj.LapStats.lapFlag =0;
            end
            % Update lap stats
            if obj.LapStats.lapFlag == 1 && obj.LapStats.prevlapFlag ==0
                if obj.LapStats.currLap > 0
                    obj.LapStats.lapTimes(obj.LapStats.currLap) = ...
                        stepresult(end,1);
                end
                if obj.LapStats.currLap == 1
                    obj.LapStats.lapResults(obj.LapStats.currLap) = ...
                        stepresult(end,1);
                elseif obj.LapStats.currLap > 1
                    obj.LapStats.lapResults(obj.LapStats.currLap) = ...
                        stepresult(end,1) - ...
                        obj.LapStats.lapTimes(obj.LapStats.currLap-1);
                end
                obj.LapStats.currLap = obj.LapStats.currLap+1;
            end
        end
        
        function [obj,tfin] = state4(obj,i,tfin,retrievedCase)
            %STATE4 Car reasoning state 4
            %   Defines functions to perform overtake
            
            Distance1 = sqrt((obj.Pose(1) - ...
                retrievedCase.waypoints(:,1)).^2 + (obj.Pose(2) - ...
                retrievedCase.waypoints(:,2)).^2);
            [~,idx1]=min(abs(Distance1));
            
            obj = obj.setLinearVelocity(retrievedCase.speed(idx1));
            obj.Controller.Waypoints = [retrievedCase.waypoints(:,1),...
                retrievedCase.waypoints(:,2)];
            stepresult = obj.simstep(i);
            % stepresult = [t y]; t -> time, y -> pose [x y rad]
            obj.TrackDistance = [obj.TrackDistance;...
                (obj.TrackDistance(end) + ...
                pdist([obj.Pose(1) obj.Pose(2); ...
                stepresult(end,2) stepresult(end,3)]))];
            obj.Pose = [stepresult(end,2) stepresult(end,3) ...
                stepresult(end,4)];
            obj.Trajectory = [obj.Trajectory; obj.Pose];
            
            tfin = [tfin;stepresult(end,1)];
            
            % Check if lap is completed
            obj.LapStats.prevlapFlag = obj.LapStats.lapFlag;
            trackArea = [0 60;17 60;17 62;0 62; 0 60];
            [in,~] = inpolygon(obj.Pose(1),obj.Pose(2),...
                trackArea(:,1),trackArea(:,2));
            if in
                obj.LapStats.lapFlag = 1;
            else
                obj.LapStats.lapFlag =0;
            end
            % Update lap stats
            if obj.LapStats.lapFlag == 1 && obj.LapStats.prevlapFlag ==0
                if obj.LapStats.currLap > 0
                    obj.LapStats.lapTimes(obj.LapStats.currLap) = ...
                        stepresult(end,1);
                end
                if obj.LapStats.currLap == 1
                    obj.LapStats.lapResults(obj.LapStats.currLap) = ...
                        stepresult(end,1);
                elseif obj.LapStats.currLap > 1
                    obj.LapStats.lapResults(obj.LapStats.currLap) = ...
                        stepresult(end,1) - ...
                        obj.LapStats.lapTimes(obj.LapStats.currLap-1);
                end
                obj.LapStats.currLap = obj.LapStats.currLap+1;
            end
        end
    end
end





