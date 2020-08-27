classdef Case
    %CASE defines case strucute
    %   Case class usef in the CBR framework, to store past experiences
    
   properties
       ID
       relativeDistance
       egoPose
       waypoints
       speed
       strategy
       successful
       laptime
       startpose
       starttime
       timereq
       prevCase
   end
   
   methods
        function cases = Case(ID, egoPose, relativeDistance)
            %CASE Constructor function
            %   Defines intial parameters of object
            if nargin > 0
                cases.ID = ID;
                cases.relativeDistance = relativeDistance;
                cases.egoPose = egoPose;
                cases.waypoints = [];
                cases.speed = [];
                cases.prevCase = [];
            end
        end
   end    
end