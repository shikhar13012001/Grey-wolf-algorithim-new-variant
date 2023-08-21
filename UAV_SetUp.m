function UAV = UAV_SetUp
% UAV_SETUP Set up parameters for a multi-UAV cooperative trajectory planning task.
% Define waypoints, threats, constraints, and cooperative settings.

% Waypoint settings
% (Each row represents parameters for a UAV)
UAV.S = [1.5, 1.5, 15;
         0,   1.5, 15;
         1.5, 0,   15;
         0,   3,   15;
         3,   0,   15;
         0,   0,   15;
         0,   4.5, 15;
         4.5, 0,   15;
         1.5, 3,   15;
         3,   1.5, 15];

UAV.G = [20, 20, 4;
         16, 20, 5;
         20, 16, 5;
         18, 20, 4.5;
         20, 18, 4.5;
         18, 18, 5;
         16, 18, 5.5;
         18, 16, 5.5;
         14, 20, 5.5;
         20, 14, 5.5];

UAV.PointNum = [30;
                26;
                24;
                26;
                24;
                30;
                24;
                24;
                24;
                24];

UAV.PointDim = size(UAV.S, 2);
UAV.num = size(UAV.S, 1);

% Threat point settings (x,y,r) or (x,y,z,r)
% (Each row represents coordinates and radius for a threat)
UAV.Menace.radar = [5, 5, 17, 2.5;
                    15, 7, 10, 2;
                    17, 18, 8, 2;
                    8, 8, 0, 6.8;
                    8, 17, 8, 3];

UAV.Menace.other = [4, 11, 12, 2.5;
                    10, 2, 10, 2;
                    5, 6, 10, 1.5;
                    10, 8, 12, 1.8;
                    11, 13, 13, 2.3;
                    13, 14, 9, 1.8;
                    18, 13, 10, 2.2;
                    16, 16, 0, 4];

% UAV constraint settings (min, max)
% (Constraints can be set individually for each UAV)
UAV.limt.v = 0.34 * repmat([0.3, 0.7], UAV.num, 1);
UAV.limt.phi = deg2rad(repmat([-60, 60], UAV.num, 1));
UAV.limt.theta = deg2rad(repmat([-45, 45], UAV.num, 1));
UAV.limt.h = repmat([0.02, 20], UAV.num, 1);
UAV.limt.x = repmat([0, 20], UAV.num, 1);
UAV.limt.y = repmat([0, 20], UAV.num, 1);
UAV.limt.z = UAV.limt.h;
UAV.limt.L = zeros(UAV.num, 2);
for i = 1:UAV.num
    zz.max = 1.6 * norm(UAV.G(i, :) - UAV.S(i, :));
    zz.min = 0.5;
    UAV.limt.L(i, :) = [zz.min, zz.max];
end

% Multi-UAV cooperative settings
UAV.tc = 160;
UAV.ds = 0.5;

% Check for errors in input parameters
ErrorCheck(UAV);
end

% Self-check for parameter consistency
function ErrorCheck(UAV)
dim = UAV.PointDim;
if dim ~= size(UAV.G, 2) || dim ~= size(UAV.Menace.radar, 2) - 1 || dim ~= size(UAV.Menace.other, 2) - 1
    if dim ~= size(UAV.G, 2)
        error('Simulation dimension is %d, but target point coordinates are %d-dimensional', dim, size(UAV.G, 2))
    else
        error('Simulation dimension is %d, but threat point coordinates are %d-dimensional', dim, size(UAV.Menace.radar, 2) - 1)
    end
end

num = UAV.num;
if num ~= size(UAV.G, 1) || num ~= size(UAV.limt.v, 1) || num ~= size(UAV.limt.phi, 1) ...
        || num ~= size(UAV.limt.theta, 1) || num ~= size(UAV.limt.h, 1) || num ~= size(UAV.limt.x, 1) ...
        || num ~= size(UAV.limt.y, 1) || num ~= size(UAV.limt.z, 1) || num ~= size(UAV.limt.L, 1)
    if num ~= size(UAV.G, 1)
        error('Number of UAVs is %d, but there are %d target points', num, size(UAV.G, 1))
    else
        error('Number of constraints does not match the number of UAVs')
    end
end

if num ~= size(UAV.PointNum, 1)
    error('Number of UAVs is %d, but navigation points are defined for %d UAVs', num, size(UAV.PointNum, 1))
end

MaxPoint = floor(UAV.limt.L(:, 2) ./ UAV.limt.L(:, 1)) - 1;
for i = 1:UAV.num
    if UAV.PointNum(i) > MaxPoint(i)
        error('Number of navigation points for UAV %d exceeds mission requirements. Try reducing the number of navigation points.', i)
    end
end
end
