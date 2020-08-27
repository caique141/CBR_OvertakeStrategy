function [path, Occmap] = Planner(startPose,goalPose,map,params,...
    PoseOpp,car1ahead,waypoints)
%PLANNER Generates a feasible path between the start and goal posa
%   This function uses the Hybrid A* algorithm to generate a path between
%   the start and goal poses.
%   It uses the opponent pose to set it as and obstacle.
%   Auxiliary obstacles are used to avoid the ego car generating a path
%   that requires backwards motion.


% Define vehicle costmap
vehicleDims = vehicleDimensions(4.5,2);
inflationRadius = 1.2;
ccConfig = inflationCollisionChecker(vehicleDims, ...
    'InflationRadius',inflationRadius);
Occmap = vehicleCostmap(map,'CellSize',0.5,'CollisionChecker',ccConfig);
opp = false;
safe = false;

% Set obstacles to avoid colision with dynamic opponent. 
if ~isempty(PoseOpp)
    xi = PoseOpp(1);
    yi = PoseOpp(2);
    theta = PoseOpp(3);

    py = [yi; yi];
    px = [xi; xi+1];
    xsafe = cos(theta) * (px-xi) - sin(theta) * (py-yi) + xi;
    ysafe = sin(theta) * (px-xi) + cos(theta) * (py-yi) + yi;
    setCosts(Occmap,[xsafe ysafe],1);
    if ~car1ahead
        py2 = [yi-1; yi+1; yi; yi; yi; yi+1 ; yi-1;  yi+1;yi-1;yi+1;...
            yi-1;yi+1;yi-1];
        px2 = [xi; xi; xi+1 ; xi+2; xi+3; xi+2 ;xi+2; xi+3;xi+3;xi+4;...
            xi+4;xi+5;xi+5];
        xsafe2 = cos(theta) * (px2-xi) - sin(theta) * (py2-yi) + xi;
        ysafe2 = sin(theta) * (px2-xi) + cos(theta) * (py2-yi) + yi;
        setCosts(Occmap,[xsafe2 ysafe2],1);
        safe = true;
    end
    opp = true;
end

% Set obstacle behind to avoid backward motion
xi = startPose(1);
yi = startPose(2);
theta = startPose(3);

py = [yi ; yi+0.5 ; yi-0.5 ; yi+1 ; yi-1];
px = [xi-3 ; xi-3 ; xi-3 ; xi-3 ; xi-3];
xn = cos(theta) * (px-xi) - sin(theta) * (py-yi) + xi;
yn = sin(theta) * (px-xi) + cos(theta) * (py-yi) + yi;
if ~isempty(waypoints)
    % Defines track limits to aviod backward motion
    [x,y] = trackSectors(startPose,waypoints);
    x = [x;xn];
    y = [y;yn];
else
    x = xn;
    y = yn;
end
setCosts(Occmap,[x y],1);

% Initialise planner and calculate waypoints
validator = validatorVehicleCostmap;
validator.Map = Occmap;
planner = plannerHybridAStar(validator,'MinTurningRadius',...
    params.minimumRadius,'MotionPrimitiveLength',...
    params.motionPrimitive,'ReverseCost',1000000);
refpath = plan(planner,startPose,goalPose);

% Remove opponent and auxiliary obstacles
setCosts(Occmap,[xn yn],0);
if opp
    setCosts(Occmap,[xsafe ysafe],0);
    if safe
        setCosts(Occmap,[xsafe2 ysafe2],0);
    end
end

% Return path planned
path = refpath;

