function solution = AGWO1(UAV, SearchAgents, Max_iter)
% GWO Gray Wolf Optimization Algorithm
% Gray Wolf Optimization

% Hyperparameters
g = 50;       % Dynamic weighting coefficient

% Algorithm Initialization
[WolfPops, Tracks] = PopsInit(UAV, SearchAgents, false);   % Randomly generate initial wolf population and trajectories
dim = WolfPops.PosDim;                                     % Dimension of state variables

% Initialize Solutions
Alpha_pos = zeros(1, dim);     % α Solution
Alpha_score = inf;             % α Solution Fitness
Alpha_no = 1;                  % α Solution Number

Beta_pos = zeros(1, dim);      % β Solution
Beta_score = inf;              % β Solution Fitness
Beta_no = 1;                   % β Solution Number

Delta_pos = zeros(1, dim);     % δ Solution
Delta_score = inf;             % δ Solution Fitness
Delta_no = 1;                  % δ Solution Number

Fitness_list = zeros(1, Max_iter);

% Iterative Solving
tic
fprintf('>>GWO Optimization in Progress    00.00%%')
for iter = 1 : Max_iter

    % ①  Calculate fitness for each wolf and update their population rank
    ProbPoints = cell(SearchAgents, 1);   
    for i = 1 : SearchAgents
        % Calculate objective function
        [fitness, ~, Data] = ObjFun(Tracks{i}, UAV);    % Objective function for one agent
        ProbPoints{i} = Data.ProbPoint;                 % States where all agents violate conditions

        % Update Alpha, Beta, and Delta solutions
        if fitness <= Alpha_score
            Alpha_score = fitness;
            Alpha_pos = WolfPops.Pos(i, :);
            Alpha_no = i;
        end 
        if fitness > Alpha_score && fitness <= Beta_score
            Beta_score = fitness;
            Beta_pos = WolfPops.Pos(i, :);
            Beta_no = i;
        end
        if fitness > Alpha_score && fitness > Beta_score && fitness <= Delta_score
            Delta_score = fitness;
            Delta_pos = WolfPops.Pos(i, :);
            Delta_no = i;
        end
    end

    % ②  Update parameter 'a'
    if abs(Fitness_list(max(1, iter-10)) - Fitness_list(iter)) < 1e-5
    a = 2 * cos((iter / Max_iter) * pi/2);   % Non-linear decrease if convergence is slow
else
    a = 2 - iter * 2 / Max_iter;              % Linear decrease
end
    %a = 2 * cos((iter / Max_iter) * pi/2);   % Non-linear decrease

    % ③  Update positions (Move towards the positions of the top three wolves)
    for i = 1 : SearchAgents
        for j = 1 : dim

            r1 = rand();
            r2 = rand();
            A1 = 2*a*r1 - a;
            C1 = 2*r2;
            D_alpha = abs(C1*Alpha_pos(j) - WolfPops.Pos(i, j));
            X1 = Alpha_pos(j) - A1*D_alpha;

            r1 = rand();
            r2 = rand();            
            A2 = 2*a*r1 - a;
            C2 = 2*r2;
            D_beta = abs(C2*Beta_pos(j) - WolfPops.Pos(i, j));
            X2 = Beta_pos(j) - A2*D_beta;
            
            r1 = rand();
            r2 = rand();
            A3 = 2*a*r1 - a;
            C3 = 2*r2;
            D_delta = abs(C3*Delta_pos(j) - WolfPops.Pos(i, j));
            X3 = Delta_pos(j) - A3*D_delta;
            
            % Static Update
            WolfPops.Pos(i, j) = (X1 + X2 + X3) / 3;
% Dynamic Update
% q = g * a; 
% if abs(Alpha_score - Delta_score) > q
%     Sum_score = Alpha_score + Beta_score + Delta_score;
%     WolfPops.Pos(i, j) = (Alpha_score*X1 + Beta_score*X2 + Delta_score*X3) / Sum_score;
% else
%     WolfPops.Pos(i, j) = (X1 + X2 + X3) / 3;
% end
        end
    end

    % ④  Adjust state variables that violate constraints
    [WolfPops, Tracks] = BoundAdjust(WolfPops, ProbPoints, UAV);

    % ⑤  Store fitness
    Fitness_list(iter) = Alpha_score;

    if iter/Max_iter*100 < 10
        fprintf('\b\b\b\b\b%.2f%%', iter/Max_iter*100)
    else
        fprintf('\b\b\b\b\b\b%.2f%%', iter/Max_iter*100)
    end
end
fprintf('\n\n>>Computation Completed!\n\n')
toc

% Output Values
solution.method = 'AGWO1';                % Algorithm
solution.WolfPops = WolfPops;           % All solution population information
solution.Tracks = Tracks;               % All solution trajectory information
solution.Fitness_list = Fitness_list;   % α solution fitness curve
solution.Alpha_Data = Data;             % Threat information of α solution
solution.Alpha_no = Alpha_no;           % Position of α solution
solution.Beta_no = Beta_no;             % Position of β solution
solution.Delta_no = Delta_no;           % Position of δ solution

end
