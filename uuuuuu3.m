%% Setup
clear, clc, close all
splittime=10000;
end_time=4;
% Define waypoints and times (example values)
%waypoints = [0.6  -0.1306  -0.5; -0.2  -0.4186 -0.5; 0.6  0.4656 0.65]; % Example waypoints
waypoints = [0.6  -0.0452 -0.5; -0.2  -0.705 -0.5; 0.6  0.6 0.65]; % Example waypoints

waypointTimes = [0 2 4]; % Time at each waypoint
% Compute direction vector from first to third waypoint
dir_vec = waypoints(:,3) - waypoints(:,1);
dir_vec = dir_vec / norm(dir_vec); % Normalize

% Set velocity at second point to 0.5 m/s in that direction
v_middle = 0.5 * dir_vec;
a_middle = 0.2 * dir_vec;
% Velocity: zero at start and end, specified at middle
waypointVels = [0      v_middle(1)  0;
                0      v_middle(2)  0;
                0      v_middle(3)  0];

% Acceleration: zero at start and end, specified at middle (set to zero for smoothness, or pick a value)
waypointAccs = [0      a_middle(1)  0;
                0      a_middle(2)  0;
                0      a_middle(3)  0];
numWaypoints = size(waypoints,2);
% Use a small sample time for this example
ts = 0.01;
trajTimes = 0:ts:waypointTimes(end);

taskOrientWaypoints =zeros(3,numel(trajTimes));
taskOrientVel = zeros(3, numel(trajTimes)); % Angular velocity for all time steps
taskOrientAcc = zeros(3, numel(trajTimes)); % Angular acceleration for all time

% Define home joint angles (example values)
jointAnglesHome = [0 0 0 0 0 0]; % Home configuration for PUMA

% Load PUMA robot model
puma = loadrobot('puma560', 'DataFormat', 'row');

% Define end effector name
eeName = 'link7';

% Define threshold for small angles
angleThreshold = 0.000001;
% Define IK
ik = inverseKinematics('RigidBodyTree',puma);
ikWeights = [0 0 0 1 1 1];
ikInitGuess =jointAnglesHome;
numJoints = numel(puma.homeConfiguration);

% Set up plot
plotMode = 1; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
show(puma, jointAnglesHome, 'Frames', 'off', 'PreservePlot', false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
if plotMode == 1
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

%% Changing Joint Limits
for idx = 1:numJoints
  puma.Bodies{idx}.Joint.PositionLimits = [-pi, pi];
end

%% Generate trajectory
% Cartesian Motion only
trajType = 'quintic';
switch trajType
    case 'trap'
        [taskPos,taskVel,taskAcc] = trapveltraj(waypoints,numel(trajTimes), ...
            'AccelTime',repmat(waypointAccelTimes,[3 1]), ... 
            'EndTime',repmat(diff(waypointTimes),[3 1]));
    case 'cubic'
        [taskPos,taskVel,taskAcc] = cubicpolytraj(waypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',zeros(3,numWaypoints));
    case 'quintic'
        [taskPos,taskVel,taskAcc] = quinticpolytraj(waypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',waypointVels, ...
            'AccelerationBoundaryCondition',waypointAccs);
    case 'bspline'
        if size(waypoints,2) < 4
        ctrlpoints = [waypoints, waypoints(:,end)];
       else
        ctrlpoints = waypoints;
       end
        [taskPos,taskVel,taskAcc] = bsplinepolytraj(ctrlpoints,waypointTimes([1 end]),trajTimes);
    case 'cubic_spline'
        sp = csape(waypointTimes, waypoints, 'clamped');
        taskPos = fnval(sp, trajTimes);
        spd = fnder(sp, 1); % Velocity
        sdd = fnder(sp, 2); % Acceleration
        taskVel = fnval(spd, trajTimes);
        taskAcc = fnval(sdd, trajTimes);
    case 'quintic_spline'
        sp = spapi(6, waypointTimes, waypoints); % 6 = order (quintic)
        taskPos = fnval(sp, trajTimes);
        spd = fnder(sp, 1); % First derivative (velocity)
        sdd = fnder(sp, 2); % Second derivative (acceleration)
        taskVel = fnval(spd, trajTimes);
        taskAcc = fnval(sdd, trajTimes);
    otherwise
        error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
end

% Show the full trajectory with the rigid body tree
if plotMode == 1
    set(hTraj,'xdata',taskPos(1,:),'ydata',taskPos(2,:),'zdata',taskPos(3,:));
    TrajPoints = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'r.-');
elseif plotMode == 2
    plotTransforms(taskPos',repmat([1 0 0 0],[size(taskPos,2) 1]),'FrameSize',0.05);
end

config_all=zeros(numel(trajTimes),6);
%% Trajectory following loop
for idx = 1:numel(trajTimes) 
    if idx==splittime
     t_replan = 0.5;
     newSecondPoint = [ 0 ; -0.605 ; 0.6]; % <--- Set your new second waypoint here
     [~, idx_current] = min(abs(trajTimes - t_replan));
     currentPos = taskPos(:, idx);
     currentVel = taskVel(:, idx);
     currentAcc = taskAcc(:, idx);
     waypoints_new = [currentPos, newSecondPoint, waypoints(:,end)];
     waypointTimes_new = [ (idx-1)*ts, 2, end_time]; % You can adjust durations as needed
     dir_vec = waypoints_new (:,3) - waypoints_new (:,1);
     dir_vec = dir_vec / norm(dir_vec); % Normalize
     v_middle = 0.5 * dir_vec;
     a_middle = 0.2 * dir_vec;
     waypointVels_new = [currentVel(1)   v_middle(1)  0;
                         currentVel(2)   v_middle(2)  0;
                         currentVel(3)   v_middle(3)  0];
     waypointAccs_new = [currentAcc(1)   0 0;
                         currentAcc(2)   0 0;
                         currentAcc(3)   0 0];
     trajTimes_new = (idx-1)*ts:ts:waypointTimes_new(end);
     [taskPos_new, taskVel_new, taskAcc_new] = quinticpolytraj(waypoints_new, waypointTimes_new, trajTimes_new, ...
         'VelocityBoundaryCondition', waypointVels_new, ...
         'AccelerationBoundaryCondition', waypointAccs_new);
     taskPos_total = [taskPos(:,1:idx-1), taskPos_new(:,1:end)];
     trajTimes_total = [trajTimes(1:idx-1),trajTimes_new(1:end)];
     break;
    end
    tgtPose = trvec2tform(taskPos(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    if strcmp(info.Status, 'success')
        ikInitGuess = config;
        config(abs(config) < angleThreshold) = 0;
        config_all(idx,:)=config;
    else
        warning('IK solver failed at time step %d', idx);
    end
    eePos = taskPos(:,idx)';
    set(TrajPoints,'xdata',[TrajPoints.XData eePos(1)], ...
               'ydata',[TrajPoints.YData eePos(2)], ...
               'zdata',[TrajPoints.ZData eePos(3)]);
    set(TrajPoints, 'Color', 'r'); % Set the trajectory color to black
    tolerance = 1e-3; % Adjust this value as needed
    isMatch = any(all(abs(waypoints' - taskPos(:,idx)') < tolerance, 2));
    if isMatch
         eeTform = getTransform(puma,config,eeName);
         plotTransforms(tform2trvec(eeTform),tform2quat(eeTform),'FrameSize',0.1);
    end
    show(puma,config_all(idx,:),'Frames','off','PreservePlot',false);
    title(['Trajectory at t = ' num2str(trajTimes(idx))])
    drawnow    
end

if splittime <=idx
for idxt = 1:1:numel(trajTimes_new) 
    tgtPose = trvec2tform(taskPos_total(:,idxt+idx-1)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    if strcmp(info.Status, 'success')
        ikInitGuess = config;
        config(abs(config) < angleThreshold) = 0;
        config_all(idxt+idx-1,:)=config;
    else
        warning('IK solver failed at time step %d', idx-1+idxt);
    end

     eePos = taskPos_total(:,idxt+idx-1)';
    set(TrajPoints,'xdata',[TrajPoints.XData eePos(1)], ...
               'ydata',[TrajPoints.YData eePos(2)], ...
               'zdata',[TrajPoints.ZData eePos(3)]);
    set(TrajPoints, 'Color', 'r'); % Set the trajectory color to black
    tolerance = 1e-3; % Adjust this value as needed
    isMatch = any(all(abs(waypoints' - taskPos_total(:,idxt+idx-1)') < tolerance, 2));
    if isMatch
         eeTform = getTransform(puma,config,eeName);
         plotTransforms(tform2trvec(eeTform),tform2quat(eeTform),'FrameSize',0.1);
    end
    show(puma,config_all(idxt+idx-1,:),'Frames','off','PreservePlot',false);
    title(['Trajectory at t = ' num2str(trajTimes(idxt)+(idx-1)*ts)])
    drawnow    
end
numSteps =size(taskPos_total,2) ;
trajTimes=trajTimes_total;
else
    numSteps = numel(trajTimes);
    taskPos_total=taskPos;
 
end


q =  config_all';    % Joint positions
q=q(:,1:size(taskPos_total,2));
qd = zeros(numJoints, numSteps);   % Joint velocities
qdd = zeros(numJoints, numSteps);  % Joint accelerations
numJoints = size(q, 1);
jointVelocities = diff(q, 1, 2) / ts;
jointVelocities = [zeros(numJoints, 1), jointVelocities]; % Pad with zeros for the first time step
jointAccelerations = diff(jointVelocities, 1, 2) / ts;
jointAccelerations = [zeros(numJoints, 1), jointAccelerations]; % Pad with zeros for the first time step
jointJerks=diff( jointAccelerations, 1, 2) / ts;
jointJerks = [zeros(numJoints, 1),  jointJerks]; % Pad with zeros for the first time step

if ~strcmp(trajType, 'bspline') && ~strcmp(trajType,'cubic_spline') 
    figure;
    plot(trajTimes, q(:,1:numel(trajTimes)), 'LineWidth', 1.5);
    title('Joint Positions');
    xlabel('Time (s)');
    ylabel('Position (rad)');
    legend(arrayfun(@(x) sprintf('Joint %d', x), 1:numJoints, 'UniformOutput', false));
    grid on;

    figure;
    plot(trajTimes, jointVelocities(:,1:numel(trajTimes)), 'LineWidth', 1.5);
    title('Joint Velocities');
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    legend(arrayfun(@(x) sprintf('Joint %d', x), 1:numJoints, 'UniformOutput', false));
    grid on;

    figure;
    plot(trajTimes, jointAccelerations(:,1:numel(trajTimes)), 'LineWidth', 1.5);
    title('Joint Accelerations');
    xlabel('Time (s)');
    ylabel('Acceleration (rad/s^2)');
    legend(arrayfun(@(x) sprintf('Joint %d', x), 1:numJoints, 'UniformOutput', false));
    grid on;

    timeVectorJerk =trajTimes(1:end);
    figure;
    plot(timeVectorJerk, jointJerks(:,1:numel(trajTimes)), 'LineWidth', 1.5);
    title('Joint-Space Jerks');
    xlabel('Time (s)');
    ylabel('Jerk (rad/s^3)');
    legend(arrayfun(@(x) sprintf('Joint %d', x), 1:numJoints, 'UniformOutput', false));
    grid on;
else
    startIdx = 10; % Start plotting from the 10th point
    figure;
    plot(trajTimes(startIdx:end), q(:,startIdx:end)', 'LineWidth', 1.5);
    title('Joint Positions (from 10th point)');
    xlabel('Time (s)');
    ylabel('Position (rad)');
    legend(arrayfun(@(x) sprintf('Joint %d', x), 1:numJoints, 'UniformOutput', false));
    grid on;

    figure;
    plot(trajTimes(startIdx:end), jointVelocities(:,startIdx:end)', 'LineWidth', 1.5);
    title('Joint Velocities (from 10th point)');
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    legend(arrayfun(@(x) sprintf('Joint %d', x), 1:numJoints, 'UniformOutput', false));
    grid on;

    figure;
    plot(trajTimes(startIdx:end), jointAccelerations(:,startIdx:end)', 'LineWidth', 1.5);
    title('Joint Accelerations (from 10th point)');
    xlabel('Time (s)');
    ylabel('Acceleration (rad/s^2)');
    legend(arrayfun(@(x) sprintf('Joint %d', x), 1:numJoints, 'UniformOutput', false));
    grid on;

    figure;
    plot(trajTimes(startIdx:end), jointJerks(:,startIdx:end)', 'LineWidth', 1.5);
    title('Joint-Space Jerks (from 10th point)');
    xlabel('Time (s)');
    ylabel('Jerk (rad/s^3)');
    legend(arrayfun(@(x) sprintf('Joint %d', x), 1:numJoints, 'UniformOutput', false));
    grid on;
end

%% Jerk Profile Analysis for All Joints (ADDED SECTION)
jerkTime = trajTimes(1,1:size(jointJerks,2)); % Ensure matching length

% 1. Plot jerk profiles for all joints
% Plot for first three joints (position)
figure;
for j = 1:3
    subplot(3,1,j);
    plot(jerkTime, jointJerks(j,:), 'b', 'LineWidth', 1.5);
    ylabel(['Joint ' num2str(j) ' Jerk']);
    grid on;
    if j == 1
        title('Joint Jerk Profiles (Position Joints 1-3)');
    end
end
xlabel('Time (s)');

% Plot for last three joints (orientation)
figure;
for j = 4:6
    subplot(3,1,j-3);
    plot(jerkTime, jointJerks(j,:), 'b', 'LineWidth', 1.5);
    ylabel(['Joint ' num2str(j) ' Jerk']);
    grid on;
    if j == 4
        title('Joint Jerk Profiles (Orientation Joints 4-6)');
    end
end

xlabel('Time (s)');

% 2. Calculate jerk metrics
maxJerk = max(abs(jointJerks), [], 2); % Max jerk per joint
rmsJerk = sqrt(mean(jointJerks.^2, 2)); % RMS jerk per joint
intAbsJerk = trapz(jerkTime, abs(jointJerks), 2); % Integrated absolute jerk per joint
intAbsJerkssum=sum(intAbsJerk(:))
% 3. Display results
fprintf('\nJerk Metrics for Each Joint:\n');
fprintf('Joint | MaxJerk   | RMSJerk   | IntAbsJerk\n');
for j = 1:numJoints
    fprintf('%5d | %9.4f | %9.4f | %12.4f\n', ...
        j, maxJerk(j), rmsJerk(j), intAbsJerk(j));
end