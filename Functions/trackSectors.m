function [x,y] = trackSectors(car1Pose,waypoints)
%TRACKSECTORS Defines track sector limits
%   This function is used to define track limits depending on the car's
%   position, to avoid backward motion.
%   Must be modified and adapted for each track used.

%   Check position of car in the reference path
Distance1 = sqrt((car1Pose(1) - waypoints(:,1)).^2 + (car1Pose(2) - ...
    waypoints(:,2)).^2);
[~,idx]=min(abs(Distance1));

%   Define sector limits behind car
if idx <= 90
    %   Sector 1
    x = [1 ; 2 ; 3 ; 4 ; 5 ; 6 ; 7 ; 8 ; 9 ; 10 ; 11 ; 12 ; 13 ; 14 ; ...
        15 ; 16 ; 17];
    y = [55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; ...
        55 ; 55 ; 55 ; 55 ; 55];
elseif idx > 90 && idx <= 180
    %   Sector 2
    x = [55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; ...
        55 ; 55 ; 55 ; 55 ; 55];
    y = [96 ; 97 ; 98 ; 99 ; 100 ; 101 ; 102 ; 103 ; 104 ; 105 ; 106 ; ...
        107 ; 108 ; 109 ; 110 ; 111 ; 112];
elseif idx > 180 && idx <= 320
    %   Sector 3
    x = [96 ; 97 ; 98 ; 99 ; 100 ; 101 ; 102 ; 103 ; 104 ; 105 ; 106 ; ...
        107 ; 108 ; 109 ; 110 ; 111 ; 112];
    y = [55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; ...
        55 ; 55 ; 55 ; 55 ; 55];
elseif idx > 320 && idx <= 440
    %   Sector 4
    x = [76 ; 77 ; 78 ; 79 ; 80 ; 81 ; 82 ; 83 ; 84 ; 85 ; 86 ; 87 ; ...
        88 ; 89 ; 90 ; 91 ; 92];
    y = [55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; ...
        55 ; 55 ; 55 ; 55 ; 55];
elseif idx > 440
    %   Sector 5
    x = [55 ; 56 ; 57 ; 58 ; 59 ; 60 ; 61 ; 62 ; 63 ; 64 ; 65 ; 66 ; ...
        67 ; 68 ; 69 ; 70 ; 71];
    y = [55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; 55 ; ...
        55 ; 55 ; 55 ; 55 ; 55];
end

end

