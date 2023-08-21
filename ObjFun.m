function [F, subF, Data] = ObjFun(Track, UAV)
%OBJFUN Objective function, fitness function (for a single agent)

% Set multi-objective optimization weights (must be a row vector)
weight = [0.05, 0.05, 0.1, 0.7, 0.7]; % Default weights

% Expression coefficient adjustment (dimensionless normalization of metrics)
p1 = 1;    % Fuel term (already divided by maximum range)
p21 = 1;   % Altitude term
p22 = 1;   % Low altitude term
p31 = 1.2; % Radar threat
p32 = 1.1; % Other threats
p4 = 1.2;  % Time synchronization term
p5 = 1;    % Collision term

% Perform trajectory detection
report = TrackDetect(Track, UAV);  % Track is a struct

% Fuel (from paper 3)
ZZ = sum(UAV.limt.L);
MaxL_mt = ZZ(2);
f_o = p1 * report.L_mt / MaxL_mt;

% Altitude (from paper 3)
dim = UAV.PointDim;
if dim < 3
    f_h = 0;
else
    f_h = 0;
    for i = 1 : UAV.num
        Hmax = UAV.limt.h(i, 2);
        Hmin = UAV.limt.h(i, 1); 
        for k = 1 : UAV.PointNum(i)
            z = Track.P{i}(3, k);
            if z > Hmax
                fk = p21 * (z - Hmax);
            elseif z >= Hmin
                fk = 0;
            else
                fk = p22 * (Hmin - z);
            end
            f_h = f_h + fk;
        end
    end
end

% Threat (from paper 2)
% Note: Only applicable to spherical or circular areas, not cylindrical areas
O_r = UAV.Menace.radar(:, 1:end-1);            % Radar            
O_o = UAV.Menace.other(:, 1:end-1);            % Missiles, artillery, weather, etc.
f_t = 0;  % Threat cost
for i = 1 : UAV.num
    for k = 1 : UAV.PointNum(i)
        P = Track.P{i}(:, k)';  % Transpose to 1*dim
        for m = 1 : size(O_r, 1)
            fk = p31 / (norm(P - O_r(m, :)))^4;
            f_t = f_t + fk;
        end
        for m = 1 : size(O_o, 1)
            fk = p32 / norm(P - O_o(m, :));
            f_t = f_t + fk;
        end
    end
end

% Synchronization (from paper 1)
f_m = 0;
for i = 1 : UAV.num
    Li = report.L(i);
    tmax = Li / UAV.limt.v(i, 1);
    tmin = Li / UAV.limt.v(i, 2);
    ti = report.time(i);
    tc = UAV.tc;
    if tc <= tmax && tc >= tmin
        fk = 0;
    else
        fk = p4 * abs(ti - tc);
    end
    f_m = f_m + fk;
end

% Collision (from paper 1)
f_c = p5 * report.col_times;

% Objective function components (must be a column vector, otherwise clustering won't work)
subF = [f_o; f_h; f_t; f_m; f_c]; % 5*1

% Weighted objective function
F = weight * subF;

% Output information
Data.ProbPoint = report.ProbPoint;    % All problematic points
Data.AngleProb = report.AngleProb;    % Points not meeting angle constraints
Data.TrajProb = report.TrajProb;      % Points not meeting minimum trajectory separation
Data.Threat = report.Threat;          % Threatened points

Data.L = report.L;                    % Range for each UAV
Data.t = report.time;                 % Time for each UAV
Data.c = report.col_times;            % Total collision count for all UAVs
end
