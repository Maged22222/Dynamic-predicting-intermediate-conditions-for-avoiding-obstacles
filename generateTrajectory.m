function [taskPos, taskVel, taskAcc] = generateTrajectory(trajType, waypoints, waypointTimes, trajTimes, waypointVels, waypointAccs, waypointAccelTimes)


    numWaypoints = size(waypoints, 2);
    if nargin < 5 || isempty(waypointVels)
        waypointVels = zeros(3, numWaypoints);
    end
    if nargin < 6 || isempty(waypointAccs)
        waypointAccs = zeros(3, numWaypoints);
    end

    switch lower(trajType)
        case 'trap'
            if nargin < 7 || isempty(waypointAccelTimes)
                
                waypointAccelTimes = diff(waypointTimes) * 0.25;
            end
            [taskPos, taskVel, taskAcc] = trapveltraj(waypoints, numel(trajTimes), ...
                'AccelTime', repmat(waypointAccelTimes, [3 1]), ... 
                'EndTime', repmat(diff(waypointTimes), [3 1]));

        case 'cubic'
            [taskPos, taskVel, taskAcc] = cubicpolytraj(waypoints, waypointTimes, trajTimes, ... 
                'VelocityBoundaryCondition', waypointVels);

        case 'quintic'
            [taskPos, taskVel, taskAcc] = quinticpolytraj(waypoints, waypointTimes, trajTimes, ... 
                'VelocityBoundaryCondition', waypointVels, ...
                'AccelerationBoundaryCondition', waypointAccs);

        case 'bspline'
            if numWaypoints < 4
                ctrlpoints = [waypoints, waypoints(:, end)];
            else
                ctrlpoints = waypoints;
            end
            [taskPos, taskVel, taskAcc] = bsplinepolytraj(ctrlpoints, waypointTimes([1 end]), trajTimes);

        otherwise
            error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
    end
end
