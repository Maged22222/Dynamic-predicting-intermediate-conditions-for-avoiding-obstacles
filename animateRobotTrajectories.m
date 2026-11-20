function animateRobotTrajectories(robot, trajTimes, configs, taskPositions, nominalPositions, waypoints, detourPoint, methodLabels, colors)


    numMethods = numel(configs);

    numSteps = numel(trajTimes);
    homeConfig = robot.homeConfiguration;

    
    for m = 1:numMethods
        currentConfig = configs{m};
        currentPos = taskPositions{m};
        methodName = methodLabels{m};
        trajColor = colors{m};

        fig = figure('Name', sprintf('Animation: %s', methodName), 'NumberTitle', 'off');
        show(robot, homeConfig, 'Frames', 'off', 'PreservePlot', false);
        hold on;
        grid on;
        xlim([-1 1]); ylim([-1 1]); zlim([0 1.2]);
        view(135, 30);

      
        if nargin >= 6 && ~isempty(waypoints)
            plot3(waypoints(1, :), waypoints(2, :), waypoints(3, :), 'ro', ...
                'LineWidth', 2, 'MarkerSize', 6, 'DisplayName', 'Nominal Waypoints');
        end

        
        if nargin >= 7 && ~isempty(detourPoint)
            plot3(detourPoint(1), detourPoint(2), detourPoint(3), 'ks', ...
                'MarkerFaceColor', 'y', 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', 'New Point');
        end

        if nargin >= 5 && ~isempty(nominalPositions) && numel(nominalPositions) >= m
            nomPos = nominalPositions{m};
            plot3(nomPos(1, :), nomPos(2, :), nomPos(3, :), ...
                'Color', trajColor, 'LineStyle', ':', 'LineWidth', 2, ...
                'DisplayName', 'Path Before Changing ');
        end

    
        hTrajTrace = plot3(currentPos(1, 1), currentPos(2, 1), currentPos(3, 1), ...
            'Color', trajColor, 'LineStyle', '-', 'LineWidth', 1.8, ...
            'DisplayName', 'Executed Path ');

        legend('Location', 'northeast', 'AutoUpdate', 'off');

        %  animation
        for idx = 1:numSteps
            if ~ishandle(fig)
                break;
            end
            set(hTrajTrace, 'XData', currentPos(1, 1:idx), ...
                            'YData', currentPos(2, 1:idx), ...
                            'ZData', currentPos(3, 1:idx));
            show(robot, currentConfig(idx, :), 'Frames', 'off', 'PreservePlot', false);
            title(sprintf('%s - Trajectory at t = %.2f s', methodName, trajTimes(idx)));
            drawnow;
        end
    end
end
