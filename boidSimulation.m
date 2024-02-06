% Initialize boids
numBoids = 10;
positions = rand(numBoids, 2) * 100; % 100x100 area
velocities = randn(numBoids, 2)/10;
idx = 1; % Calculate alignment for the first boid
visibleRange = 10; % Boids within 10 units are considered neighbors
T = 1000;
borderBuffer = 10;
minVelocity = 0.5; 
maxVelocity = 1.5;
separationDistance = 10;
borderDistance = 30; % Adjust as needed
borderFollowingFactor = 0.1; % Adjust the strength of border following
        
% Adjust these factors to control the speed
% alignmentFactor = 0.25; % Reduce if they move too fast
% velocityDamping = .75; % Apply damping to slow down overall speed


for t = 1:T % For each time step
    for i = 1:numBoids
        % Calculate the three rules
        v1 = alignment(positions, velocities, i, visibleRange);%* alignmentFactor;
        v2 = cohesion(positions, i, visibleRange); % Cohesion
        v3 = separation(positions, i, separationDistance); % Separation
        
        % Calculate the border following force
        v4 = borderFollowing(positions, i, borderDistance, borderFollowingFactor);
    
    
        % Update boid velocity and position
%         velocities(i, :) = velocities(i, :) + v1;
%         positions(i, :) = positions(i, :) + velocities(i, :);
        velocities(i, :) = velocities(i, :) + v1 + v2 + v3; %* velocityDamping;
        if norm(velocities(i, :)) < minVelocity
            velocities(i, :) = (velocities(i, :) / norm(velocities(i, :))) * minVelocity;
        end
        if norm(velocities(i, :)) > maxVelocity
            velocities(i, :) = (velocities(i, :) / norm(velocities(i, :))) * maxVelocity;
        end
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

function vCohesion = cohesion(positions, idx, visibleRange)
    [neighbors, ~] = findNeighbors(positions, idx, visibleRange);
    if isempty(neighbors)
        vCohesion = [0, 0];
    else
        % Calculate the center of mass of neighbors
        centerOfMass = mean(positions(neighbors, :), 1);
        % Steer towards the center of mass
        vCohesion = (centerOfMass - positions(idx, :)) * 0.01; % Adjusting factor
    end
end

function vSeparation = separation(positions, idx, separationDistance)
    vSeparation = [0, 0];
    for j = 1:size(positions, 1)
        if j ~= idx
            distance = norm(positions(idx, :) - positions(j, :));
            if distance < separationDistance
                vSeparation = vSeparation + (positions(idx, :) - positions(j, :)) / distance;
            end
        end
    end
    vSeparation = vSeparation * 0.1; % Adjusting factor for separation strength
end

function vBorderFollowing = borderFollowing(positions, idx, borderDistance, borderFollowingFactor)
    % Calculate the distance of the boid from the center
    center = [50, 50]; % Assuming the center of the simulation area is at (50, 50)
    distanceToCenter = norm(positions(idx, :) - center);
    
    % Determine if the boid is too far from the border
    if distanceToCenter > borderDistance
        % Calculate a vector that steers the boid towards the border
        vBorderFollowing = (center - positions(idx, :)) / norm(center - positions(idx, :)) * borderFollowingFactor;
    else
        % If the boid is close enough to the border, no action needed
        vBorderFollowing = [0, 0];
    end
end

