function [WolfPops, Tracks] = BoundAdjust(WolfPops, ProbPoints, UAV)
%BOUNDADJUST Constraint Handling
Tracks = Pops2Tracks(WolfPops, UAV);

% ① Constraints Not Satisfied: "Delete" trajectory points by taking the average of neighboring points
dim = UAV.PointDim;                                    % Dimension
for agent = 1 : size(WolfPops.Pos, 1)
    Track = Tracks{agent};                             % Trajectory (struct structure)
    ProbPoint = ProbPoints{agent};                     % Problematic points
    Position = [];                                     % New wolf pack encoding
    for i = 1:UAV.num
          PointNum = UAV.PointNum(i);
          % Delete problematic points on a trajectory
          for k = 1 : PointNum
                flag = ProbPoint{i}(k);
                if flag == 1
                    if k == 1
                        P1 = UAV.S(i, :)' ;
                    else
                        P1 = Track.P{i}(:, k-1);
                    end
                    if k == PointNum
                        P2 = UAV.G(i, :)' ;
                    else
                        P2 = Track.P{i}(:, k+1);
                    end
                    Track.P{i}(:, k) = (P1+P2) / 2;  % Delete trajectory point
                end
          end
    
          % Convert to wolf pack encoding format
          p = Track.P{i} ;
          p = reshape(p, 1, dim * PointNum);
          Position = [Position, p];
    end
    V = Track.V';
    Position = [Position, V];
    
    % New position information
    WolfPops.Pos(agent, :) = Position;
end

% ② Boundary Handling: Clip values that exceed the boundaries
WolfPops.Pos = BoundClamp(WolfPops.Pos, WolfPops.lb, WolfPops.ub);

% Generate new trajectories
Tracks = Pops2Tracks(WolfPops, UAV);

end

% Boundary Clamping
function x = BoundClamp(x, lb, ub)
    Flag4ub = x > ub;
    Flag4lb = x < lb;
    x = x .* ( ~(Flag4ub + Flag4lb) ) + ub .* Flag4ub + lb .* Flag4lb;
end
