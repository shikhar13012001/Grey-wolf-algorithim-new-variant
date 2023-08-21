function solution = MP_GWO(UAV, SearchAgents, Max_iter)
%MP_GWO Multi-Population Gray Wolf Optimization
%Multi Population Gray Wolf Optimization

% Hyperparameter
g = 50;       % Dynamic update weight coefficient

% Algorithm initialization
[WolfPops, ~] = PopsInit(UAV, SearchAgents, false);   % Randomly generate initial wolf population
ClassPops = PopsCluster(WolfPops, UAV);               % Perform initial clustering (to obtain k value)
dim = WolfPops.PosDim;                                % Dimension of state variables
cSearchAgents = ClassPops.SearchAgents;               % Number of search agents (sub-population count)
SearchAgents = cSearchAgents * ClassPops.k;           % Adjust the total number of search agents (multiple of k)
WolfPops.Pos = WolfPops.Pos(1:SearchAgents, :);       % Modify the population to match the adjusted count

% Error handling
if cSearchAgents < 4
    error('Too few search agents')
end 

% Initialize solutions
Alpha_pos = zeros(ClassPops.k, dim);          % α solution
Alpha_score = 1 ./ zeros(ClassPops.k, 1);     % α solution fitness

Beta_pos = zeros(ClassPops.k, dim);            % β solution
Beta_score = 1 ./ zeros(ClassPops.k, 1);       % β solution fitness

Delta_pos = zeros(ClassPops.k, dim);           % δ solution
Delta_score = 1 ./ zeros(ClassPops.k, 1);      % δ solution fitness

Fitness_list = zeros(ClassPops.k, Max_iter);  % Fitness curve

Pops.PosDim = WolfPops.PosDim;  % Sub-population
Pops.lb = WolfPops.lb;
Pops.ub = WolfPops.ub;

% Iterative solution
tic
fprintf('>>MP-GWO Optimization    00.00%%')
for iter = 1 : Max_iter

    % ① Update parameter 'a'
    a = 2 - iter * 2 / Max_iter;          % Linear decrease
    %a = 2 * cos((iter/Max_iter)*pi/2);   % Non-linear decrease
    
    % ② Cluster
    if iter > 1 % Initial clustering is already done, saving computation
        ClassPops = PopsCluster(WolfPops, UAV);
    end
    
    % Iterate through clusters
    for k = 1 : ClassPops.k
        Positions = ClassPops.Pos{k};  

        % ③ Find α, β, δ wolves
        for i = 1 : cSearchAgents
            % Read fitness
            fitness = ClassPops.Fitness(k, i);

            % Update Alpha, Beta, and Delta solutions
            if fitness <= Alpha_score(k)  % The strongest fitness (smaller is better)
                Alpha_score(k) = fitness;
                Alpha_pos(k, :) = Positions(i, :);
            end 
            if fitness > Alpha_score(k) && fitness <= Beta_score(k)
                Beta_score(k) = fitness;
                Beta_pos(k, :) = Positions(i, :);
            end
            if fitness > Alpha_score(k) && fitness > Beta_score(k) && fitness <= Delta_score(k)
                Delta_score(k) = fitness;
                Delta_pos(k, :) = Positions(i, :);
            end
        end

        % ④ Update positions (towards the positions of the top three wolves)
        for i = 1 : cSearchAgents
            for j = 1 : dim
                r1 = rand();
                r2 = rand();
                A1 = 2 * a * r1 - a;
                C1 = 2 * r2;
                D_alpha = abs(C1 * Alpha_pos(k, j) - Positions(i, j));
                X1 = Alpha_pos(k, j) - A1 * D_alpha;
    
                r1 = rand();
                r2 = rand();            
                A2 = 2 * a * r1 - a;
                C2 = 2 * r2;
                D_beta = abs(C2 * Beta_pos(k, j) - Positions(i, j));
                X2 = Beta_pos(k, j) - A2 * D_beta;
                
                r1 = rand();
                r2 = rand();
                A3 = 2 * a * r1 - a;
                C3 = 2 * r2;
                D_delta = abs(C3 * Delta_pos(k, j) - Positions(i, j));
                X3 = Delta_pos(k, j) - A3 * D_delta;
                
                % Static update
                %Positions(i, j) = (X1 + X2 + X3) / 3;
                % Dynamic update
                                q = g * a; % Threshold
                                if abs(Alpha_score(k) - Delta_score(k)) > q
                                    Sum_score = Alpha_score(k) + Beta_score(k) + Delta_score(k);
                                    Positions(i, j) = (Alpha_score(k) * X1 + Beta_score(k) * X2 + Delta_score(k) * X3) / Sum_score;
                                else
                                    Positions(i, j) = (X1 + X2 + X3) / 3;
                                end
            end
        end

    % ⑤ Adjust state variables that do not meet the requirements
    Pops.Pos = Positions;
    ProbPoints =  ClassPops.ProbPoints{k};
    [Pops, ~] = BoundAdjust(Pops, ProbPoints, UAV);

    % ⑥ Store fitness
    Fitness_list(k, iter) = Alpha_score(k);

    % ⑦ Merge populations
    WolfPops.Pos(cSearchAgents * (k - 1) + 1 : cSearchAgents * k, :) = Pops.Pos;

    end

    if iter / Max_iter * 100 < 10
        fprintf('\b\b\b\b\b%.2f%%', iter / Max_iter * 100)
    else
        fprintf('\b\b\b\b\b\b%.2f%%', iter / Max_iter * 100)
    end
end
fprintf('\n\n>>Calculation completed!\n\n')
toc

% Find positions of α, β, δ
n = 3; 
A = ClassPops.Fitness; 
t = findmin(A, n);
index = cSearchAgents * (t(:, 1) - 1) + t(:, 2);

real_Alpha_no = index(1);
real_Beta_no = index(2);
real_Delta_no = index(3);
Alpha_Data = ClassPops.Data{t(1, 1)}{t(1, 2)}  ;

% Output
solution.method = 'MP-GWO';                                % Algorithm
% solution.ClassPops = ClassPops;                          % Classification information
solution.WolfPops = WolfPops;                              % All population information
solution.Tracks = Pops2Tracks(WolfPops, UAV);              % All trajectory information
solution.Fitness_list = mean(Fitness_list, 1);             % Average fitness curve of all α solutions
solution.Alpha_Data = Alpha_Data;                          % Threat information of real α
solution.Alpha_no = real_Alpha_no;                         % Position of real α
solution.Beta_no = real_Beta_no;                           % Position of real β
solution.Delta_no = real_Delta_no;                         % Position of real δ

end

%% Find the positions of the smallest n values in matrix A
function t = findmin(A, n)
    t = sort(A(:));
    [x, y] = find(A <= t(n), n);
    t = [x, y];         % Positions of the smallest n items in matrix A [row, column]
    B = zeros(n, 1);
    for i = 1 : n
        B(i) = A(t(i, 1), t(i, 2));
    end
    [~, index] = sort(B);
    t = t(index, :); % Positions of the smallest n items sorted from small to large
end

%% Cluster the population
function [ClassPops] = PopsCluster(WolfPops, UAV)

SearchAgents = size(WolfPops.Pos, 1);  % Number of search agents 
Dim = WolfPops.PosDim;                % Dimension of search agents
Tracks = Pops2Tracks(WolfPops, UAV); % Convert search agents to trajectory information

% Calculate fitness
o_Fitness = zeros(SearchAgents, 1); % 60*1
o_subF = []; % 5*60
o_ProbPoints = cell(SearchAgents, 1);  % 60*1
o_Data = cell(SearchAgents, 1); % 60*1

% Parallel computing of fitness
for i = 1:SearchAgents
    [fitness, subF, Data] = ObjFun(Tracks{i}, UAV);
    o_ProbPoints{i} = Data.ProbPoint;  %cell-cell
    o_Data(i) = {Data}; %cell-struct
    o_Fitness(i) = fitness; %vector-var
    o_subF = [o_subF, subF]; 
end

% Clustering
k = size(subF, 1);                          % k clusters (determined by ObjFun)
cSearchAgents = floor(SearchAgents / k);    % Number of agents per cluster
cFitness = zeros(k, cSearchAgents);         % Store fitness for each cluster
cPositions = cell(k, 1);                    % Store positions for each cluster
cTracks = cell(k, 1);                       % Store trajectory information for each cluster
cProbPoints = cell(k, 1);                   % Store problematic trajectory points for each cluster
cData = cell(k, 1);                         % Store detection reports for each cluster

% Sort
[~, Index] = sort(o_subF, 2, "ascend") ;     % Sort along dimension 2 in ascending order
                                             % Smaller fitness is better
% Cluster
for i = 1:k
    Positions = zeros(cSearchAgents, Dim);
    batchTrack = cell(cSearchAgents, 1);
    batchProbPoints = cell(cSearchAgents, 1);
    batchData = cell(cSearchAgents, 1);
    for j = 1:cSearchAgents
        idx = Index(i, j);
        cFitness(i, j) = o_Fitness(idx); %mat-vector
        Positions(j, :) = WolfPops.Pos(idx, :); %mat-mat
        batchTrack{j} = Tracks{idx}; %cell-cell
        batchProbPoints{j} = o_ProbPoints{idx}; %cell-cell
        batchData{j} = o_Data{idx}; %cell-cell
    end
    cPositions(i) = {Positions}; %cell-mat
    cTracks{i} = batchTrack; %cell-cell
    cProbPoints{i} = batchProbPoints; %cell-cell
    cData{i} = batchData; %cell-cell
end

% Output
ClassPops.Pos = cPositions;
ClassPops.Tracks = cTracks;
ClassPops.ProbPoints = cProbPoints;
ClassPops.Data = cData;
ClassPops.Fitness = cFitness;
ClassPops.SearchAgents = cSearchAgents;
ClassPops.k = k;
end
