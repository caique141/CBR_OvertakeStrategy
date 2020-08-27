function [car1ahead,aux] = checkCarAhead(totalWaypoints,car1,...
    car2,shiftedposes)
%CHECKCARAHEAD Check if the car1 is ahead of car2
%   This function uses the shifted poses, having the car1 as reference. 
%   And the car2 pose is used to check the distance between them.

n = length(shiftedposes);

%   Retrieve position of car2 in reference of car1
Distance2 = sqrt((car2.Pose(1) - shiftedposes(:,1)).^2 + ...
    (car2.Pose(2) - shiftedposes(:,2)).^2);
[~,idx2]=min(abs(Distance2));

%   Check if car2 is at least 1/4 of lap behind car1, if true car1 is set
%   as ahead of car2
aux = n-idx2;
if n-2>idx2 && idx2>ceil(3*n/4)
    car1ahead = true;
else
    car1ahead = false;
end

