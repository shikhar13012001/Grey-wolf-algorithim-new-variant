function solution = CS_GWO(UAV, SearchAgents, Max_iter)
%CS_GWO Gray Wolf Cuckoo Optimization Algorithm
%Gray Wolf Cuckoo Optimization

% Hyperparameters
pa = 0.25;   % Cuckoo search parameter

% Algorithm Initialization
[WolfPops, Tracks] = PopsInit(UAV, SearchAgents, false);   % Randomly generate initial wolf population and trajectories
dim = WolfPops.PosDim;                                    % Dimension of state variables

% Initialize Solutions
Alpha_pos = zeros(1, dim);   % α Solution
Alpha_score = inf;                  % α Solution Fitness
Alpha_no = 1;                         % α Solution Number

Beta_pos = zeros(1, dim);      % β Solution
Beta_score = inf;                     % β Solution Fitness
Beta_no = 1;                             % β Solution Number

Delta_pos = zeros(1, dim);     % δ Solution
Delta_score = inf;                    % δ Solution Fitness
Delta_no = 1;                           % δ Solution Number

Fitness_list = zeros(1, Max_iter);

% Iterative Solving
tic
fprintf('>>CS_GWO Optimization in Progress    00.00%%')
for iter = 1 : Max_iter

    % ①  Calculate fitness for each wolf and update their population rank
    ProbPoints = cell(SearchAgents, 1);   
    for i = 1 : SearchAgents
        % Calculate objective function
        [fitness, ~, Data] = ObjFun(Tracks{i}, UAV);    % Objective function for one agent
        ProbPoints{i} = Data.ProbPoint;                       % States where all agents violate conditions

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
    a = 2 - iter * 2 / Max_iter;                  % Linear decrease
    %a = 2 * cos((iter / Max_iter) * pi/2);   % Non-linear decrease

    % ③  Update positions (Move towards the positions of the top three wolves)
    for i = 1 : SearchAgents
        for j = 1 : dim

            r1 = rand();
            r2 = rand();
            A1 = 2*a*r1 - a;
            C1 = 2*r2;
            D_alpha = abs(C1*Alpha_pos(j) - WolfPops.Pos(i, j));
            X1(i, j) = Alpha_pos(j) - A1*D_alpha;

            r1 = rand();
            r2 = rand();            
            A2 = 2*a*r1 - a;
            C2 = 2*r2;
            D_beta = abs(C2*Beta_pos(j) - WolfPops.Pos(i, j));
            X2(i, j) = Beta_pos(j) - A2*D_beta;
            
            r1 = rand();
            r2 = rand();
            A3 = 2*a*r1 - a;
            C3 = 2*r2;
            D_delta = abs(C3*Delta_pos(j) - WolfPops.Pos(i, j));
            X3(i, j) = Delta_pos(j) - A3*D_delta;
            
            % Updating commented out
            %WolfPops.Pos(i, j) = (X1(i, j) + X2(i, j) + X3(i, j)) / 3;
        end
    end

    % ④  Cuckoo search
    fitness = nan(SearchAgents, 1);
    for i = 1 : SearchAgents
        [fitness(i), ~, ~] = ObjFun(Tracks{i}, UAV);
    end
    [~, index] = min(fitness);
    best = WolfPops.Pos(index, :);
    X1 = get_cuckoos(X1, best, WolfPops.lb, WolfPops.ub); 
    X2 = get_cuckoos(X2, best, WolfPops.lb, WolfPops.ub);
    X3 = get_cuckoos(X3, best, WolfPops.lb, WolfPops.ub);
    X1 = empty_nests(X1, WolfPops.lb, WolfPops.ub, pa);
    X2 = empty_nests(X2, WolfPops.lb, WolfPops.ub, pa);
    X3 = empty_nests(X3, WolfPops.lb, WolfPops.ub, pa);
    WolfPops.Pos = (X1 + X2 + X3) / 3;

    % ⑤  Adjust state variables that violate constraints
    [WolfPops, Tracks] = BoundAdjust(WolfPops, ProbPoints, UAV);

    % ⑥  Store fitness
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
solution.method = 'GWO';                % Algorithm
solution.WolfPops = WolfPops;       % All solution population information
solution.Tracks = Tracks;                  % All solution trajectory information
solution.Fitness_list = Fitness_list;   % α solution fitness curve
solution.Alpha_Data = Data;            % Threat information of α solution
solution.Alpha_no = Alpha_no;        % Position of α solution
solution.Beta_no = Beta_no;             % Position of β solution
solution.Delta_no = Delta_no;          % Position of δ solution

end

%%%%% Cuckoo Search %%%%%
function nest = get_cuckoos(nest, best, Lb, Ub)
    n = size(nest, 1);
    beta = 3/2;
    sigma = (gamma(1 + beta) * sin(pi*beta/2)/(gamma((1 + beta)/2)*beta*2^((beta - 1)/2)))^(1/beta);
    for j = 1:n
        s = nest(j, :);
        u = randn(size(s))*sigma;
        v = randn(size(s));
        step = u./abs(v).^(1/beta);
        stepsize = 0.01*step.*(s - best);
        s = s + stepsize.*randn(size(s));
        nest(j, :) = BoundClamp(s, Lb, Ub);
    end
end

function new_nest = empty_nests(nest, Lb, Ub, pa)
    n = size(nest, 1);
    K = rand(size(nest)) > pa;
    stepsize = rand*(nest(randperm(n), :) - nest(randperm(n), :));
    new_nest = nest + stepsize.*K;
    for j = 1:size(new_nest, 1)
        s = new_nest(j, :);
        new_nest(j, :) = BoundClamp(s, Lb, Ub);
    end
end

function x = BoundClamp(x, lb, ub)
    Flag4ub = x > ub;
    Flag4lb = x < lb;
    x = x .* ( ~(Flag4ub + Flag4lb) ) + ub .* Flag4ub + lb .* Flag4lb;
end
