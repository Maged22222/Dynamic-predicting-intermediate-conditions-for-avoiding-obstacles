function config_all = solveTrajectoryIK(ik, eeName, ikWeights, ikInitGuess, taskPos, angleThreshold)


    if nargin < 6 || isempty(angleThreshold)
        angleThreshold = 1e-6;
    end

    numSteps = size(taskPos, 2);
    numJoints = numel(ikInitGuess);
    config_all = zeros(numSteps, numJoints);
    currentGuess = ikInitGuess;

    for idx = 1:numSteps
        tgtPose = trvec2tform(taskPos(:, idx)');
        [config, info] = ik(eeName, tgtPose, ikWeights, currentGuess);
        if strcmp(info.Status, 'success')
            currentGuess = config;
            config(abs(config) < angleThreshold) = 0;
            config_all(idx, :) = config;
        else
            warning('IK solver failed at step %d', idx);
            config_all(idx, :) = currentGuess;
        end
    end
end
