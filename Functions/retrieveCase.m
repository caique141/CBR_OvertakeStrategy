function [retrievedCase,val] = retrieveCase(CaseBase,car1,...
    waypoints,noise)
%RETRIEVECASE Retrieve case from data set
%   Retrieves cases from the given data set based on the similarity level

%   Shift the waypoints, with the ego car position as reference
%   Get ego car position index
if ~isempty(noise)
Distance1 = sqrt((car1.Pose(1)+noise(1) - waypoints(:,1)).^2 + ...
    (car1.Pose(2)+noise(2) - waypoints(:,2)).^2);
else
Distance1 = sqrt((car1.Pose(1) - waypoints(:,1)).^2 + (car1.Pose(2) - ...
    waypoints(:,2)).^2);
end
[~,idx1]=min(abs(Distance1));

%   Shift waypoints and track distance with index reference
shiftedwaypoints = circshift(waypoints,-(idx1-1));

%   Calculate distance from car to waypoints
shifteddistance = [0];
for i=2:length(shiftedwaypoints)
    shifteddistance = [shifteddistance ; (shifteddistance(end) + ...
        pdist([shiftedwaypoints(i,1) shiftedwaypoints(i,2);...
        shiftedwaypoints(i-1,1) shiftedwaypoints(i-1,2)]))];
end

%   Get case starting points
Distance1 = [];
Distance2 = [];
idx = NaN(length(CaseBase),1);
for i=1:length(CaseBase)
    
    Distance1 = sqrt((CaseBase(i).startpose(1) - ...
        shiftedwaypoints(:,1)).^2 + (CaseBase(i).startpose(2) - ...
        shiftedwaypoints(:,2)).^2);
    [~,idxstart]=min(abs(Distance1));
    
    Distance2 = sqrt((CaseBase(i).egoPose(1) - ...
        shiftedwaypoints(:,1)).^2 + (CaseBase(i).egoPose(2) - ...
        shiftedwaypoints(:,2)).^2);
    [~,idxego]=min(abs(Distance2));
    idx(i) = min([idxstart,idxego]);
end

%   Retrieve five closest cases
retry = true;
while retry
    [values,I]=sort(idx);
    newvector = CaseBase(I(1:5));
    
    vectorID = zeros(length(newvector),1);
    previousCases = [];
    auxiliaryindex = [];
    for i = 1:length(newvector)
        if ~isempty(newvector(i).prevCase)
            previousCases = [previousCases; newvector(i).prevCase];
            auxiliaryindex = [auxiliaryindex; i];
        end
        vectorID(i) = newvector(i).ID;
    end
    sum=0;
    if ~isempty(previousCases)
        for k = 1:length(previousCases)
            if ismember(previousCases(k),vectorID)
                if auxiliaryindex(k)<find(vectorID==previousCases(k))
                    idx(previousCases(k))=1000;
                else
                    idx(newvector(auxiliaryindex(k)).ID)=1000;
                end
                sum = sum+1;
            end
        end
    end
    if sum>0
        retry = true;
    else
        retry = false;
    end
end

Alltimes = [];
for i=1:length(newvector)
    Alltimes = [Alltimes; newvector(i).timereq];
end

%   Define retrieving costs
gain = 0.12; % gain=0.05
DistanceFromCase = sqrt((gain*values(1:length(newvector))).^2 + ...
    (Alltimes).^2);

%   Get case with minimum cost
[~,idx]=min(DistanceFromCase);
retrievedCase = newvector(idx);
val = values(idx);
end