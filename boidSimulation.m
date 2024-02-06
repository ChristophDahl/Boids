% Initialize boids
numBoids = 10;
positions = rand(numBoids, 2) * 100; % 100x100 area
velocities = randn(numBoids, 2);
idx = 1; % Calculate alignment for the first boid
visibleRange = 10; % Boids within 10 units are considered neighbors
T = 1000;
borderBuffer = 10;

% Adjust these factors to control the speed
alignmentFactor = 0.05; % Reduce if they move too fast
velocityDamping = 0.95; % Apply damping to slow down overall speed


for t = 1:T % For each time step
    for i = 1:numBoids
        % Calculate the three rules
        v1 = alignment(positions, velocities, idx, visibleRange)* alignmentFactor;

%         v2 = rule2(positions, i); % Cohesion
%         v3 = rule3(positions, i); % Separation
        
        % Update boid velocity and position
%         velocities(i, :) = velocities(i, :) + v1;
%         positions(i, :) = positions(i, :) + velocities(i, :);
        velocities(i, :) = (velocities(i, :) + v1) * velocityDamping;
        positions(i, :) = positions(i, :) + velocities(i, :);
        
        % Border avoidance
        if positions(i, 1) < borderBuffer
            velocities(i, 1) = abs(velocities(i, 1)); % Turn right
        elseif positions(i, 1) > 100 - borderBuffer
            velocities(i, 1) = -abs(velocities(i, 1)); % Turn left
        end
        if positions(i, 2) < borderBuffer
            velocities(i, 2) = abs(velocities(i, 2)); % Turn up
        elseif positions(i, 2) > 100 - borderBuffer
            velocities(i, 2) = -abs(velocities(i, 2)); % Turn down
        end
        
    end
    
    % Visualization
    plot(positions(:, 1), positions(:, 2), 'b.');
    xlim([0, 100]);
    ylim([0, 100]);
    drawnow;
end


function vAlignment = alignment(positions, velocities, idx, visibleRange)
    % Find neighbors
    [neighbors, distances] = findNeighbors(positions, idx, visibleRange);
    if isempty(neighbors)
        vAlignment = [0, 0];
    else
        % Calculate average velocity of neighbors
        avgVelocity = mean(velocities(neighbors, :), 1);
        % Adjust velocity towards the average
        vAlignment = (avgVelocity - velocities(idx, :)) * 0.05; % Adjusting factor
    end
end
function [neighbors distancesSquared] = findNeighbors(positions, idx, visibleRange)
    % Calculate the squared distances from the current boid to all others
    distancesSquared = sum((positions - positions(idx,:)).^2, 2);
    % Identify neighbors within the visible range, excluding the current boid itself
    isNeighbor = distancesSquared > 0 & distancesSquared < visibleRange^2;
    neighbors = find(isNeighbor);
end
