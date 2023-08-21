function [report] = TrackDetect(Track, UAV)
%TRACKDETECT Determine if the trajectory meets the criteria (for a single agent)

%% UAV Trajectory Detection

% Threat detection
dim = UAV.PointDim;                          % Simulation environment dimension
M = [UAV.Menace.radar; UAV.Menace.other];    % Threat regions

Threat = cell(UAV.num, 1);                   % Threat results tree
Angle = cell(UAV.num, 1);                    % Angle detection results tree
MiniTraj = cell(UAV.num, 1);                 % Minimum trajectory segment detection results tree
ProbPoint = cell(UAV.num, 1);                % Problematic points

L = cell(UAV.num, 1);                        % Trajectory segment tree (cumulative structure)
Time = cell(UAV.num, 1);                     % Arrival times at various points

L_mt = 0;                                    % Sum of all UAV trajectories
totalTime = zeros(UAV.num, 1);               % Time used
totalL = zeros(UAV.num, 1);                  % Total distance

for i = 1 : UAV.num
    PointNum = UAV.PointNum(i);   
    Judge = zeros(1, PointNum+1);
    L_i = zeros(1, PointNum+1);
    Time_i = zeros(1, PointNum+1);
    Angle_i = zeros(1, PointNum+1);
    Traj_i = zeros(1, PointNum+1);
    ProbPoint_i = zeros(1, PointNum+1);
    
    l = 0;
    V = Track.V(i);
    
    for k = 1 : PointNum
        P2 = Track.P{i}(:, k)';   % Transpose to 1*dim
        if k == 1
            P1 = UAV.S(i, :);
            ZZ = P2 - P1;
            phi0 = atan2(ZZ(2), ZZ(1));   % Angle detection
            phi1 = phi0;
            d_phi = phi1 - phi0;
            if dim > 2
                theta0 = atan(ZZ(3) / sqrt(ZZ(1)^2 + ZZ(2)^2));  % Inclination detection
                theta1 = theta0;
                d_theta = theta1 - theta0;
            else
                d_theta = 0;
            end
        else
            P1 = Track.P{i}(:, k-1)';  % Transpose to 1*dim
            ZZ = P2 - P1;
            phi1 = atan2(ZZ(2), ZZ(1));
            d_phi = phi1 - phi0;
            phi0 = phi1;
            
            if dim > 2
                theta1 = atan(ZZ(3) / sqrt(ZZ(1)^2 + ZZ(2)^2));
                d_theta = theta1 - theta0;
                theta0 = theta1;
            else
                d_theta = 0;
            end
        end
        
        [across, ~] = CheckThreat(P1, P2, M);  % Threat detection
        Judge(k) = across;
        
        dl = norm(P1 - P2);
        l = l + dl; % Accumulate distance
        t = l / V;  % Time point
        L_i(k) = l;
        Time_i(k) = t;

        if abs(d_phi) > UAV.limt.phi(i, 2)  ||  abs(d_theta) > UAV.limt.theta(i, 2)
            Angle_i(k) = true;
        else
            Angle_i(k) = false;
        end

        if dl < UAV.limt.L(i, 1)
            Traj_i(k) = true;
        else
            Traj_i(k) = false;
        end

        ProbPoint_i(k) = Angle_i(k) | Traj_i(k) | Judge(k); % Problematic points

        % Last segment detection
        if k == PointNum
            P1 = UAV.G(i, :);
            [across, ~] = CheckThreat(P2, P1, M);
            Judge(k+1) = across;
            
            dl = norm(P1 - P2);
            l = l + dl;
            t = l / V;
            L_i(k+1) = l;
            Time_i(k+1) = t;

            ZZ = P1 - P2;
            phi1 = atan2(ZZ(2), ZZ(1));
            d_phi = phi1 - phi0;
            if dim > 2
                theta1 = atan(ZZ(3) / sqrt(ZZ(1)^2 + ZZ(2)^2));
                d_theta = theta1 - theta0;
            else
                d_theta = 0;
            end

            if abs(d_phi) > UAV.limt.phi(i, 2)  ||  abs(d_theta) > UAV.limt.theta(i, 2)
                Angle_i(k+1) = true;
            else
                Angle_i(k+1) = false;
            end

            if dl < UAV.limt.L(i, 1)
                Traj_i(k+1) = true;
            else
                Traj_i(k+1) = false;
            end

            ProbPoint_i(k+1) = Angle_i(k+1) | Traj_i(k+1) | Judge(k+1);
        end
    end   
    
    Threat(i) = {Judge};          % Detection results (length is one more than the number of points)
    Angle(i) = {Angle_i};         % Angles
    MiniTraj(i) = {Traj_i};       % Minimum trajectory intervals
    ProbPoint(i) = {ProbPoint_i}; % Problematic points

    L(i) = {L_i};                 % Path length (cumulative)
    Time(i) = {Time_i};           % Time (cumulative)
    
    L_mt = L_mt + l;              % Total length
    totalTime(i) = t;             % Time
    totalL(i) = l;                % Length
end

% Multi-UAV collision detection
d_safe = UAV.ds;                 % Safety distance
CollideTimes = 0;                % Collision count

for i = 2 : UAV.num
    PointNum_i = UAV.PointNum(i);
    
    for k = 1 : PointNum_i
        P1 = Track.P{i}(:, k)';  % K time i UAV position
        t_i = Time{i, 1}(k);     % K time i UAV time
        
        for j = 1 : i-1
            PointNum_j = UAV.PointNum(j);
            flag = false; % 
            % Search points at the same time
            for kj = 1 : PointNum_j
                if kj == 1
                    t_j_l = 0;
                    P_l = UAV.S(j, :); 
                else
                    t_j_l = Time{j}(kj-1);
                    P_l = Track.P{j}(:, kj-1)'; 
                end
                
                t_j_r = Time{j}(kj);
                P_r = Track.P{j}(:, kj)'; 

                if t_i <= t_j_r  &&  t_i >= t_j_l
                    flag = true;
                    P2 =  P_l + (t_i - t_j_l) / (t_j_r - t_j_l) * (P_r - P_l);  % K time J UAV position
                end
            end
            
            if flag  % Found P2 position, perform collision detection
                collide = CheckCollide(P1, P2, d_safe);
            else     % No P2 position, no collision
                collide = false;
            end
            
            if collide
                CollideTimes = CollideTimes + 1;
            end
        end
    end
end

% Generate detection report
report.L_mt = L_mt;               % Total travel sum
report.Threat = Threat;           % Threatened trajectory point positions
report.AngleProb = Angle;         % Points with unsatisfactory angles
report.TrajProb = MiniTraj;       % Points with unsatisfactory minimum trajectory intervals
report.ProbPoint = ProbPoint;     % Points with problems
report.L = totalL;                % Flight distance
report.time = totalTime;          % Flight time
report.col_times = CollideTimes;  % Number of collisions
end

%% Crossing Threat Detection
function [across, across_num] = CheckThreat(P1, P2, M)
    % Threat regions (spheres or circular areas, not suitable for cylindrical areas)
    O = M(:,1:end-1);  % Center
    R = M(:, end);     % Radius 
    
    % Check if the line segment passes through a certain restricted area
    total = 0;
    for i = 1 : size(O, 1)
        a = norm(P1 - P2);
        b = norm(P2 - O(i, :));
        c = norm(P1 - O(i, :));
        
        % Is point P1 inside the circle
        if c < R(i)         
            isHit = true;   
        % Is point P2 inside the circle
        elseif b < R(i)  
            isHit = true;   
        else
            dim = size(O, 2);  % P1: 1*dim dimensions
            if dim < 3
                % Planar case   
                A = P1(2) - P2(2);
                B = P2(1) - P1(1);
                C = P1(1) * P2(2) - P2(1) * P1(2);
                x = O(i, 1);
                y = O(i, 2);
                d = abs(A * x + B * y + C) / sqrt(A^2 + B^2); 
            else
                % Spatial case
                PP = P2 - P1;
                PO = O(i, :) - P1;
                d = norm(cross(PP, PO)) / a;
            end
            
            % Distance criterion
            if d >= R(i)
                isHit = false;  % No intersection
            % Angle criterion (intersection when distance is satisfied and both angles are acute)
            elseif d > 0
                cosP1 = a^2 + c^2 - b^2 / (2 * c * a);
                cosP2 = a^2 + b^2 - c^2 / (2 * b * a);
                
                if cosP1 > 0  &&  cosP2 > 0
                    isHit = true;
                else
                    isHit = false;
                end
            % Both points are outside, distance is 0 (collinear case)
            else
                if a > b  &&  a > c
                    isHit = true;
                else
                    isHit = false;
                end
            end
        end

        if isHit
            total = total + 1;  % Total collisions
        end
    end

    if total > 0
        across = true;
    else
        across = false;     % Whether it passes through the restricted area
    end
    across_num = total;     % Number of times passing through the restricted area
end

%% Collision Detection
function [collide] = CheckCollide(P1, P2, d_safe)
    if norm(P1 - P2) >= d_safe
        collide = false;
    else
        collide = true;
    end
end
