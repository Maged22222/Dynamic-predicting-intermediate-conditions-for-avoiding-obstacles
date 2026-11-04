%% Setup
clear, clc, close all

end_time = 4;
splittime = (end_time / 4) * 100; % Time index to trigger obstacle avoidance (t = 1.0 s)
ts = 0.01;

% Waypoints and timing
waypoints = [0.6  -0.0452 -0.5; -0.2  -0.705 -0.5; 0.6  0.6 0.65];
waypointTimes = [0, end_time / 2, end_time];
trajTimes = 0:ts:waypointTimes(end);
numWaypoints = size(waypoints, 2);

% Robot configuration
puma = loadrobot('puma560', 'DataFormat', 'row');
eeName = 'link7';
numJoints = numel(puma.homeConfiguration);
jointAnglesHome = [0 0 0 0 0 0];
angleThreshold = 1e-6;

% Joint limits
for idx = 1:numJoints
    puma.Bodies{idx}.Joint.PositionLimits = [-pi, pi];
end

% IK solver 
ik = inverseKinematics('RigidBodyTree', puma);
ikWeights = [0 0 0 1 1 1]; 

plotMode = 1; 
trajType = 'quintic';
newSecondPoint = [0; -0.605; 0.6]; % Obstacle avoidance waypoint
% initial conditions
dir_vec = waypoints(:,3) - waypoints(:,1);
dir_vec = dir_vec / norm(dir_vec);
v_middle_avg = 0.5 * dir_vec;
a_middle_avg = 0.5 * dir_vec;

waypointVels_avg = [0, v_middle_avg(1), 0;
                    0, v_middle_avg(2), 0;
                    0, v_middle_avg(3), 0];
waypointAccs_avg = [0, a_middle_avg(1), 0;
                    0, a_middle_avg(2), 0;
                    0, a_middle_avg(3), 0];

%%  Average Method

% Initial trajectory
[taskPos_avg, taskVel_avg, taskAcc_avg] = generateTrajectory(trajType, waypoints, waypointTimes, trajTimes, waypointVels_avg, waypointAccs_avg);

currentPos_avg = taskPos_avg(:, splittime);
currentVel_avg = taskVel_avg(:, splittime);
currentAcc_avg = taskAcc_avg(:, splittime);

waypoints_new_avg = [currentPos_avg, newSecondPoint, waypoints(:,end)];
waypointTimes_new = [(splittime - 1) * ts, 2, end_time];
trajTimes_new = (splittime - 1) * ts:ts:waypointTimes_new(end);

dir_vec_new = waypoints_new_avg(:,3) - waypoints_new_avg(:,1);
dir_vec_new = dir_vec_new / norm(dir_vec_new);
v_middle_new_avg = 0.5 * dir_vec_new;
a_middle_new_avg = 0.5 * dir_vec_new;

waypointVels_new_avg = [currentVel_avg(1), v_middle_new_avg(1), 0;
                        currentVel_avg(2), v_middle_new_avg(2), 0;
                        currentVel_avg(3), v_middle_new_avg(3), 0];
waypointAccs_new_avg = [currentAcc_avg(1), a_middle_new_avg(1), 0;
                        currentAcc_avg(2), a_middle_new_avg(2), 0;
                        currentAcc_avg(3), a_middle_new_avg(3), 0];

[taskPos_new_avg, ~, ~] = generateTrajectory(trajType, waypoints_new_avg, waypointTimes_new, trajTimes_new, ...
    waypointVels_new_avg, waypointAccs_new_avg);

taskPos_total_avg = [taskPos_avg(:, 1:splittime - 1), taskPos_new_avg];

%%  Dynamic Predicting Method

waypointVels_dyn = waypointVels_avg;
waypointAccs_dyn = waypointAccs_avg;

% Initial trajectory
[taskPos_dyn, taskVel_dyn, taskAcc_dyn] = generateTrajectory(trajType, waypoints, waypointTimes, trajTimes, waypointVels_dyn, waypointAccs_dyn);

currentPos_dyn = taskPos_dyn(:, splittime);
currentVel_dyn = taskVel_dyn(:, splittime);
currentAcc_dyn = taskAcc_dyn(:, splittime);

waypoints_new_dyn = [currentPos_dyn, newSecondPoint, waypoints(:,end)];

[v_middle_new_dyn, a_middle_new_dyn] = computeIntermediateVelAcc(waypoints_new_dyn(:,1), waypoints_new_dyn(:,2), waypoints_new_dyn(:,3), ...
    waypointTimes_new(1), waypointTimes_new(2), waypointTimes_new(3));

waypointVels_new_dyn = [currentVel_dyn(1), v_middle_new_dyn(1), 0;
                        currentVel_dyn(2), v_middle_new_dyn(2), 0;
                        currentVel_dyn(3), v_middle_new_dyn(3), 0];
waypointAccs_new_dyn = [currentAcc_dyn(1), a_middle_new_dyn(1), 0;
                        currentAcc_dyn(2), a_middle_new_dyn(2), 0;
                        currentAcc_dyn(3), a_middle_new_dyn(3), 0];

[taskPos_new_dyn, ~, ~] = generateTrajectory(trajType, waypoints_new_dyn, waypointTimes_new, trajTimes_new, ...
    waypointVels_new_dyn, waypointAccs_new_dyn);

taskPos_total_dyn = [taskPos_dyn(:, 1:splittime - 1), taskPos_new_dyn];
trajTimes_total = [trajTimes(1:splittime - 1), trajTimes_new];
numSteps = size(taskPos_total_avg, 2);

%%  Inverse Kinematics for Both Methods

config_all_avg = solveTrajectoryIK(ik, eeName, ikWeights, jointAnglesHome, taskPos_total_avg, angleThreshold);
config_all_dyn = solveTrajectoryIK(ik, eeName, ikWeights, jointAnglesHome, taskPos_total_dyn, angleThreshold);


%% 4. Animation for Both Methods

if plotMode == 1
    methodLabels = {'Average Method', 'Dynamic Predicting Method'};
    configs = {config_all_avg, config_all_dyn};
    taskPositions = {taskPos_total_avg, taskPos_total_dyn};     % Path AFTER changing (actual executed path)
    nominalPositions = {taskPos_avg, taskPos_dyn};             % Path BEFORE changing (dotted line)
    animateRobotTrajectories(puma, trajTimes_total, configs, taskPositions, nominalPositions, ...
        waypoints, newSecondPoint, methodLabels, {'b', 'r'});
end


%% Kinematics (Velocity, Acceleration, Jerk)

% Average Method Kinematics
q_avg = config_all_avg';
qd_avg = [zeros(numJoints, 1), diff(q_avg, 1, 2) / ts];
qdd_avg = [zeros(numJoints, 1), diff(qd_avg, 1, 2) / ts];
jointJerks_avg = [zeros(numJoints, 1), diff(qdd_avg, 1, 2) / ts];

% Dynamic Predicting Method Kinematics
q_dyn = config_all_dyn';
qd_dyn = [zeros(numJoints, 1), diff(q_dyn, 1, 2) / ts];
qdd_dyn = [zeros(numJoints, 1), diff(qd_dyn, 1, 2) / ts];
jointJerks_dyn = [zeros(numJoints, 1), diff(qdd_dyn, 1, 2) / ts];


%%  Plot Joint-Space Trajectories (Comparative Overlay)

labels = {'Average Method', 'Dynamic Predicting Method'};
plotJointSpaceTrajectories(trajTimes_total, {q_avg, q_dyn}, {qd_avg, qd_dyn}, {qdd_avg, qdd_dyn}, ...
    [], waypointTimes, labels, {'b', 'r'}, {jointJerks_avg, jointJerks_dyn});

%% Jerk Metrics Calculation and Comparison

compareJerkMetrics(trajTimes_total, jointJerks_avg, jointJerks_dyn, {'Avg', 'Dyn'});


