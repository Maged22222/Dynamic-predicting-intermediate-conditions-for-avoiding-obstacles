function plotJointSpaceTrajectories(trajTimes, q, qd, qdd, jointWaypoints, waypointTimes, titleSuffix, lineColor, qddd)


    isComp = iscell(qd) || iscell(q) || iscell(qdd);

    if isComp
       
        if nargin >= 7 && iscell(titleSuffix)
            labels = titleSuffix;
            titleSuffix = '';
        elseif nargin >= 7 && ischar(titleSuffix) && ~isempty(titleSuffix)
            labels = {'Method 1', 'Method 2'};
            titleSuffix = ['  ', titleSuffix];
        else
            labels = {'Numerical Differentiation', 'Inverse Jacobian'};
            titleSuffix = '';
        end

        color1 = 'b';
        color2 = 'r';
        style1 = '-';
        style2 = '--';
        if nargin >= 8 && iscell(lineColor) && numel(lineColor) >= 2
            color1 = lineColor{1};
            color2 = lineColor{2};
        end

       
        if iscell(q)
            q1 = q{1};
            q2 = q{2};
            sharedQ = false;
        else
            q1 = q;
            q2 = q;
            sharedQ = true;
        end

        qd1 = qd{1};
        qd2 = qd{2};

        qdd1 = qdd{1};
        qdd2 = qdd{2};

        hasJerk = (nargin >= 9 && ~isempty(qddd));
        if hasJerk
            if iscell(qddd)
                qddd1 = qddd{1};
                qddd2 = qddd{2};
            else
                qddd1 = qddd;
                qddd2 = [];
            end
        end

        numJoints = size(q1, 1);
        numWaypoints = numel(waypointTimes);

        numSubplots = 3;
        if hasJerk
            numSubplots = 4;
        end

        for i = 1:numJoints
            figure('Name', sprintf('Joint %d Comparison', i), 'NumberTitle', 'off');

            subplot(numSubplots, 1, 1);
            hold on;
            hPos1 = plot(trajTimes, q1(i, :), 'Color', color1, 'LineStyle', style1, 'LineWidth', 1.5);
            if ~sharedQ
                hPos2 = plot(trajTimes, q2(i, :), 'Color', color2, 'LineStyle', style2, 'LineWidth', 1.5);
                legend([hPos1, hPos2], labels{1}, labels{2}, 'Location', 'best', 'AutoUpdate', 'off');
            else
                legend(hPos1, 'Joint Position', 'Location', 'best', 'AutoUpdate', 'off');
            end
            for wIdx = 1:numWaypoints
                xline(waypointTimes(wIdx), 'k--', 'HandleVisibility', 'off');
            end
            if nargin >= 5 && ~isempty(jointWaypoints)
                if iscell(jointWaypoints)
                    if ~isempty(jointWaypoints{1})
                        plot(waypointTimes, jointWaypoints{1}(i, :), 'bo', 'MarkerSize', 5, 'LineWidth', 1.2, 'HandleVisibility', 'off');
                    end
                    if numel(jointWaypoints) >= 2 && ~isempty(jointWaypoints{2})
                        plot(waypointTimes, jointWaypoints{2}(i, :), 'ro', 'MarkerSize', 5, 'LineWidth', 1.2, 'HandleVisibility', 'off');
                    end
                else
                    plot(waypointTimes, jointWaypoints(i, :), 'ro', 'MarkerSize', 4, 'HandleVisibility', 'off');
                end
            end
            title(['Joint ', num2str(i), titleSuffix, ' Position']);
            xlabel('Time (s)');
            ylabel('Position (rad)');
            grid on;

            % Joint Velocity
            subplot(numSubplots, 1, 2);
            hold on;
            plot(trajTimes, qd1(i, :), 'Color', color1, 'LineStyle', style1, 'LineWidth', 1.5);
            plot(trajTimes, qd2(i, :), 'Color', color2, 'LineStyle', style2, 'LineWidth', 1.5);
            for wIdx = 1:numWaypoints
                xline(waypointTimes(wIdx), 'k--', 'HandleVisibility', 'off');
            end
            title(['Joint ', num2str(i), titleSuffix, ' Velocity']);
            xlabel('Time (s)');
            ylabel('Velocity (rad/s)');
            grid on;

            %  Joint Acceleration
            subplot(numSubplots, 1, 3);
            hold on;
            plot(trajTimes, qdd1(i, :), 'Color', color1, 'LineStyle', style1, 'LineWidth', 1.5);
            plot(trajTimes, qdd2(i, :), 'Color', color2, 'LineStyle', style2, 'LineWidth', 1.5);
            for wIdx = 1:numWaypoints
                xline(waypointTimes(wIdx), 'k--', 'HandleVisibility', 'off');
            end
            title(['Joint ', num2str(i), titleSuffix, ' Acceleration']);
            xlabel('Time (s)');
            ylabel('Acceleration (rad/s^2)');
            grid on;

            % Joint Jerk 
            if hasJerk
                subplot(numSubplots, 1, 4);
                hold on;
                plot(trajTimes, qddd1(i, :), 'Color', color1, 'LineStyle', style1, 'LineWidth', 1.5);
                if ~isempty(qddd2)
                    plot(trajTimes, qddd2(i, :), 'Color', color2, 'LineStyle', style2, 'LineWidth', 1.5);
                end
                for wIdx = 1:numWaypoints
                    xline(waypointTimes(wIdx), 'k--', 'HandleVisibility', 'off');
                end
                title(['Joint ', num2str(i), titleSuffix, ' Jerk']);
                xlabel('Time (s)');
                ylabel('Jerk (rad/s^3)');
                grid on;
            end
        end

    else
       
        if nargin < 7 || isempty(titleSuffix)
            titleSuffix = '';
        else
            titleSuffix = ['  ', titleSuffix];
        end

        if nargin < 8 || isempty(lineColor)
            lineColor = 'b';
        end

        numJoints = size(q, 1);
        numWaypoints = numel(waypointTimes);

        hasJerk = (nargin >= 9 && ~isempty(qddd));
        if hasJerk && size(qddd, 1) ~= numJoints && size(qddd, 2) == numJoints
            qddd = qddd';
        end

        numSubplots = 3;
        if hasJerk
            numSubplots = 4;
        end

        for i = 1:numJoints
            figure;

            % Plot joint position 
            subplot(numSubplots, 1, 1);
            hold on;
            hTraj = plot(trajTimes, q(i, :), 'Color', lineColor, 'LineWidth', 1.5);
            for wIdx = 1:numWaypoints
                xline(waypointTimes(wIdx), 'k--', 'HandleVisibility', 'off');
            end
            if nargin >= 5 && ~isempty(jointWaypoints)
                hWp = plot(waypointTimes, jointWaypoints(i, :), 'ro', 'MarkerSize', 4);
                legend([hTraj, hWp], 'Trajectory', 'Waypoints', 'AutoUpdate', 'off');
            else
                legend(hTraj, 'Trajectory', 'AutoUpdate', 'off');
            end
            title(['Joint ', num2str(i), titleSuffix, ' Position']);
            xlabel('Time (s)');
            ylabel('Position (rad)');
            grid on;

            % Plot joint velocity
            subplot(numSubplots, 1, 2);
            hold on;
            plot(trajTimes, qd(i, :), 'Color', lineColor, 'LineWidth', 1.5);
            for wIdx = 1:numWaypoints
                xline(waypointTimes(wIdx), 'k--', 'HandleVisibility', 'off');
            end
            title(['Joint ', num2str(i), titleSuffix, ' Velocity']);
            xlabel('Time (s)');
            ylabel('Velocity (rad/s)');
            grid on;

            % Plot joint acceleration
            subplot(numSubplots, 1, 3);
            hold on;
            plot(trajTimes, qdd(i, :), 'Color', lineColor, 'LineWidth', 1.5);
            for wIdx = 1:numWaypoints
                xline(waypointTimes(wIdx), 'k--', 'HandleVisibility', 'off');
            end
            title(['Joint ', num2str(i), titleSuffix, ' Acceleration']);
            xlabel('Time (s)');
            ylabel('Acceleration (rad/s^2)');
            grid on;

            % Plot joint jerk
            if hasJerk
                subplot(numSubplots, 1, 4);
                hold on;
                plot(trajTimes, qddd(i, :), 'Color', lineColor, 'LineWidth', 1.5);
                for wIdx = 1:numWaypoints
                    xline(waypointTimes(wIdx), 'k--', 'HandleVisibility', 'off');
                end
                title(['Joint ', num2str(i), titleSuffix, ' Jerk']);
                xlabel('Time (s)');
                ylabel('Jerk (rad/s^3)');
                grid on;
            end
        end
    end
end
