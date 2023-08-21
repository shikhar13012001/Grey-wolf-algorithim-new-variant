function [UAV] = UAV_SetUp2
% UAV_SETUP2 Set up parameters for a multi-UAV cooperative trajectory planning task
% Environment from Paper 1

% Waypoint settings
% (Each row represents parameters for a UAV)
UAV.S = [0,   0;];

UAV.G = [875, 875;];

UAV.PointNum = [26;];

UAV.PointDim = size(UAV.S, 2);
UAV.num = size(UAV.S, 1);

% Threat point settings (x,y,r) or (x,y,z,r)
% (Each row represents coordinates and radius for a threat)
UAV.Menace.radar = [200, 200, 20;
                    600, 700, 20];

UAV.Menace.other = [80,  40,  40;
                    300, 300, 40;
                    350, 600, 40;
                    480, 450, 20;
                    700, 700, 40;
                    720, 760, 20;
                    680, 760, 20;
                    200, 400, 40;
                    200, 520, 40;
                    200, 600, 20;
                    500, 200, 40;
                    700, 200, 40];

% UAV constraint settings (min, max)
% (Constraints can be set individually for each UAV)
UAV.limt.v = 0.34 * repmat([0.3, 0.7], UAV.num, 1);
UAV.limt.phi = deg2rad(repmat([-60, 60], UAV.num, 1));
UAV.limt.theta = deg2rad(repmat([-45, 45], UAV.num, 1));
UAV.limt.h = repmat([0.02, 20], UAV.num, 1);
UAV.limt.x = repmat([0, 875], UAV.num, 1);
UAV.limt.y = repmat([0, 875], UAV.num, 1);
UAV.limt.z = UAV.limt.h;
UAV.limt.L = zeros(UAV.num, 2);
for i = 1:UAV.num
    zz.max = 1.5 * norm(UAV.G(i, :) - UAV.S(i, :));
    zz.min = 2;
    UAV.limt.L(i, :) = [zz.min, zz.max];
end

% Multi-UAV cooperative settings
UAV.tc = 6850;
UAV.ds = 25;

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
