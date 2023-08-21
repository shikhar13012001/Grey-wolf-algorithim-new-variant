% Define the positions of the starting point and the goal point
S = [0, 0];   % Starting point
G = [19, 10]; % Goal point

% Define the number of iterations and the number of wolves (potential solutions)
num_iterations = 500;
num_wolves = 10;

% Initialize the positions of the wolves randomly within the search space
wolves = rand(num_wolves, 2) * 10;

% Initialize the best solution (alpha)
alpha = [20,20];

% Iterate through generations
for iteration = 1:num_iterations
    % Evaluate the cost (fitness) for each wolf's position
    costs = zeros(num_wolves, 1);
    for i = 1:num_wolves
        % Calculate Euclidean distance from the wolf's position to the goal
        costs(i) = norm(wolves(i, :) - G);
    end
    
    % Update the alpha wolf if a better solution is found
    [~, best_wolf_idx] = min(costs);
    if costs(best_wolf_idx) < norm(alpha - G)
        alpha = wolves(best_wolf_idx, :);
    end
    
    % Update the positions of the wolves using the GWO update equation
    for i = 1:num_wolves
        A = 2 - 2 * iteration / num_iterations; % Exploration coefficient
        D_alpha = abs(2 * alpha - wolves(i, :));
        wolves(i, :) = (alpha - A .* D_alpha) + rand(1, 2) .* (wolves(i, :) - wolves(best_wolf_idx, :));
        
        % Ensure the positions are within bounds
        wolves(i, :) = max(wolves(i, :), 0);
        wolves(i, :) = min(wolves(i, :), 10);
    end
end

% The optimal path is represented by the alpha wolf's position and the goal
optimal_path = [alpha; G];
disp(optimal_path);
