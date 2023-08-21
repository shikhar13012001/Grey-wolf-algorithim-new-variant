function [WolfPops, Tracks] = PopsInit(UAV, SearchAgents, GreedyInit)
%POPSINIT Initialize the population "positions"
if (nargin < 3)
    GreedyInit = false;
end

% Self-check
if SearchAgents < 4
    error('Number of search agents is too small')
end

dim = UAV.PointDim;  % Dimension of the coordinate axes 

% Velocity for n UAVs
lb_v = UAV.limt.v(:, 1)';    % 1*n
ub_v = UAV.limt.v(:, 2)';    % 1*n
V0 = rand(SearchAgents, UAV.num) .* repmat(ub_v - lb_v, SearchAgents, 1) + repmat(lb_v, SearchAgents, 1);

% Trajectory points
P0 = [];
for i = 1 : UAV.num
    lb = [UAV.limt.x(i, 1), UAV.limt.y(i, 1), UAV.limt.z(i, 1)];     % 1*3 dimensions
    ub = [UAV.limt.x(i, 2), UAV.limt.y(i, 2), UAV.limt.z(i, 2)];    % 1*3 dimensions
    if dim < 3
        lb = lb(1:2);      % 1*2 dimensions
        ub = ub(1:2);      % 1*2 dimensions
    end
    PointNum = UAV.PointNum(i);  % Number of trajectory points
    P_i = rand(SearchAgents, dim*PointNum) .* repmat(ub - lb, SearchAgents, PointNum) + repmat(lb, SearchAgents, PointNum);
    
    % Greedy generation (generate greedy points from the second trajectory point)
    if GreedyInit
        for k = 2 : PointNum    
            ep = 0.2;      % Threshold
            m = rand();    % Mutation quantity
            if m >= ep     % When the mutation parameter exceeds the threshold, the k-th trajectory point is greedily generated; otherwise, it's generated randomly
                lb = P_i(:, dim*(k-1)-dim+1:dim*(k-1));                 % N*2 dimensions
                ub = repmat(UAV.G(i, :), SearchAgents, 1);               % N*2 dimensions
                P_i(:, dim*k-dim+1:dim*k) = rand(SearchAgents, dim) .* (ub - lb) + lb;
            end
        end
    end

    P0 = [P0, P_i];
end

WolfPops.Pos = [P0, V0]; % Wolf population positions

% Upper and lower limits for wolf population positions
lb = [];
ub = [];
for i = 1 : UAV.num
    lb_i = [UAV.limt.x(i, 1), UAV.limt.y(i, 1), UAV.limt.z(i, 1)];     % 1*3 dimensions
    ub_i = [UAV.limt.x(i, 2), UAV.limt.y(i, 2), UAV.limt.z(i, 2)];    % 1*3 dimensions
    if dim < 3
        lb_i = lb_i(1:2);      % 1*2 dimensions
        ub_i = ub_i(1:2);      % 1*2 dimensions
    end
    PointNum = UAV.PointNum(i);
    lb_i = repmat(lb_i, 1, PointNum);
    ub_i = repmat(ub_i, 1, PointNum);
    
    lb = [lb, lb_i];
    ub = [ub, ub_i];
end
lb = [lb, lb_v];
ub = [ub, ub_v];

WolfPops.PosDim = size(lb, 2);
WolfPops.lb = lb;
WolfPops.ub = ub;

% Generate initial tracks
Tracks = Pops2Tracks(WolfPops, UAV);

clc
fprintf('>>Algorithm initialization completed!\n\n')

end
