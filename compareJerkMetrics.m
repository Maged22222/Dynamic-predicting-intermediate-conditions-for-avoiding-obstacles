function [metrics1, metrics2] = compareJerkMetrics(trajTimes, jointJerks1, jointJerks2, methodLabels)


    if iscell(jointJerks1)
        if nargin >= 3 && iscell(jointJerks2)
            methodLabels = jointJerks2;
        end
        tempJerks = jointJerks1;
        jointJerks1 = tempJerks{1};
        jointJerks2 = tempJerks{2};
    end

    label1 = 'Avg';
    label2 = 'Dyn';
    if nargin >= 4 && iscell(methodLabels) && numel(methodLabels) >= 2
        label1 = methodLabels{1};
        label2 = methodLabels{2};
    elseif nargin >= 3 && iscell(jointJerks2) && numel(jointJerks2) >= 2
        label1 = jointJerks2{1};
        label2 = jointJerks2{2};
    end

    maxJerk_avg = max(abs(jointJerks1), [], 2);
    rmsJerk_avg = sqrt(mean(jointJerks1.^2, 2));
    intAbsJerk_avg = trapz(trajTimes, abs(jointJerks1), 2);

    maxJerk_dyn = max(abs(jointJerks2), [], 2);
    rmsJerk_dyn = sqrt(mean(jointJerks2.^2, 2));
    intAbsJerk_dyn = trapz(trajTimes, abs(jointJerks2), 2);

    numJoints = size(jointJerks1, 1);

    
    fprintf('\n==========================================================================================\n');
    fprintf('                             JERK METRICS COMPARISON TABLE                                \n');
    fprintf('==========================================================================================\n');
    fprintf('Joint |   Max Jerk (%3s / %3s)   |   RMS Jerk (%3s / %3s)   |  IntAbsJerk (%3s / %3s)     \n', ...
        label1, label2, label1, label2, label1, label2);
    fprintf('------------------------------------------------------------------------------------------\n');
    for j = 1:numJoints
        fprintf('  %d   | %9.4f / %9.4f   | %9.4f / %9.4f   | %11.4f / %11.4f\n', ...
            j, maxJerk_avg(j), maxJerk_dyn(j), ...
            rmsJerk_avg(j), rmsJerk_dyn(j), ...
            intAbsJerk_avg(j), intAbsJerk_dyn(j));
    end
    fprintf('------------------------------------------------------------------------------------------\n');
    fprintf('TOTAL | IntAbsJerk Sum (%s): %11.4f  |  IntAbsJerk Sum (%s): %11.4f\n', ...
        label1, sum(intAbsJerk_avg), label2, sum(intAbsJerk_dyn));
    fprintf('==========================================================================================\n');

    if nargout >= 1
        metrics1.MaxJerk = maxJerk_avg;
        metrics1.RMSJerk = rmsJerk_avg;
        metrics1.IntAbsJerk = intAbsJerk_avg;
        metrics1.TotalIntAbsJerk = sum(intAbsJerk_avg);
    end
    if nargout >= 2
        metrics2.MaxJerk = maxJerk_dyn;
        metrics2.RMSJerk = rmsJerk_dyn;
        metrics2.IntAbsJerk = intAbsJerk_dyn;
        metrics2.TotalIntAbsJerk = sum(intAbsJerk_dyn);
    end
end
