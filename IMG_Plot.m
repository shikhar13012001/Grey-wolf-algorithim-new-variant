function IMG_Plot(solution, UAV)
%IMG_PLOT Plotting Function (Requires manual addition of UAVs)
close all; 

% Solutions
Tracks = solution.Tracks;                 % Trajectories
Data = solution.Alpha_Data;               % Optimal trajectory information
Fitness_list = solution.Fitness_list;     % Fitness curve
Alpha_no = solution.Alpha_no;             % α Solution number
Beta_no = solution.Beta_no;               % β Solution number
Delta_no = solution.Delta_no;             % γ Solution number
agent_no = Alpha_no;                      % Number of the solution to be plotted


% Trajectory Plot
if UAV.PointDim < 3

    %%%%%%%%% ———— 2D Simulation ———— %%%%%%%%%
    
    x = cell(UAV.num, 1);
    y = cell(UAV.num, 1);
    for i = 1:UAV.num
        x{i} = [UAV.S(i,1), Tracks{agent_no, 1}.P{i, 1}(1,:), UAV.G(i,1)];
        y{i} = [UAV.S(i,2), Tracks{agent_no, 1}.P{i, 1}(2,:), UAV.G(i,2)];
    end

    figure(1)
    for i = 1:UAV.num
        plot(x{i}, y{i}, 'LineWidth', 1)  % Modify this line
        hold on
    end
    for i = 1:UAV.num
        plot(UAV.S(i,1), UAV.S(i,2), 'ko', 'LineWidth', 1, 'MarkerSize', 9)
        hold on
        plot(UAV.G(i,1), UAV.G(i,2), 'p', 'color', 'k', 'LineWidth', 1, 'MarkerSize', 10)
        hold on
    end
    for i = 1:size(UAV.Menace.radar,1)
        rectangle('Position', [UAV.Menace.radar(i,1)-UAV.Menace.radar(i,3), UAV.Menace.radar(i,2)-UAV.Menace.radar(i,3), 2*UAV.Menace.radar(i,3), 2*UAV.Menace.radar(i,3)], 'Curvature', [1,1], 'EdgeColor', 'k', 'FaceColor', 'g')
        hold on
    end
    for i = 1:size(UAV.Menace.other,1)
        rectangle('Position', [UAV.Menace.other(i,1)-UAV.Menace.other(i,3), UAV.Menace.other(i,2)-UAV.Menace.other(i,3), 2*UAV.Menace.other(i,3), 2*UAV.Menace.other(i,3)], 'Curvature', [1,1], 'EdgeColor', 'k', 'FaceColor', 'c')
        hold on
    end
    for i = 1:UAV.num
        leg_str{i} = ['Track', num2str(i)];  
    end
    leg_str{UAV.num+1} = 'Start';
    leg_str{UAV.num+2} = 'End';
    legend(leg_str)
    grid on
    axis equal
    xlim([-25, 900])    % Modify this line
    ylim([-25, 900])    % Modify this line
    xlabel('x(km)')
    ylabel('y(km)')
    title('Path Planning Plot')
else

    %%%%%%%%% ———— 3D Simulation ———— %%%%%%%%%

    x = cell(UAV.num, 1);
    y = cell(UAV.num, 1);
    z = cell(UAV.num, 1);
    for i = 1:UAV.num
        x{i} = [UAV.S(i,1), Tracks{agent_no, 1}.P{i, 1}(1,:), UAV.G(i,1)];
        y{i} = [UAV.S(i,2), Tracks{agent_no, 1}.P{i, 1}(2,:), UAV.G(i,2)];
        z{i} = [UAV.S(i,3), Tracks{agent_no, 1}.P{i, 1}(3,:), UAV.G(i,3)];
    end

    figure(1)
    for i = 1:UAV.num
        plot3(x{i}, y{i}, z{i}, 'LineWidth', 2)   % Modify this line
        hold on
    end
    for i = 1:UAV.num
        plot3(UAV.S(i,1), UAV.S(i,2), UAV.S(i,3), 'ko', 'LineWidth', 1.3, 'MarkerSize', 12)
        hold on
        plot3(UAV.G(i,1), UAV.G(i,2), UAV.G(i,3), 'p', 'color', 'k', 'LineWidth', 1.3, 'MarkerSize', 13)
        hold on
    end
    for i = 1:size(UAV.Menace.radar,1)
        drawsphere(UAV.Menace.radar(i,1), UAV.Menace.radar(i,2), UAV.Menace.radar(i,3), UAV.Menace.radar(i,4), true)
        hold on
    end
    for i = 1:size(UAV.Menace.other,1)
        drawsphere(UAV.Menace.other(i,1), UAV.Menace.other(i,2), UAV.Menace.other(i,3), UAV.Menace.other(i,4))
        hold on
    end
    for i = 1:UAV.num
        leg_str{i} = ['Track', num2str(i)];  
    end
    leg_str{UAV.num+1} = 'Start';
    leg_str{UAV.num+2} = 'End';
    legend(leg_str)
    grid on
    axis square
    %axis equal
    xlim([-25, 900])    % Modify this line
    ylim([-25, 900])    % Modify this line
    zlim([0, 25])       % Modify this line
    xlabel('x(km)')
    ylabel('y(km)')
    zlabel('z(km)')
    title('Path Planning Plot')

end

% Fitness
figure(2)
plot(Fitness_list, 'k', 'LineWidth', 1)
grid on
xlabel('iter')
ylabel('fitness')
title('Fitness Curve')

% Screen Output
fprintf('\nNumber of UAVs: %d', UAV.num)
fprintf('\nNumber of Navigation Points per UAV:')
fprintf('%d,  ', UAV.PointNum)
fprintf('\nUAV Flight Distance:')
fprintf('%.2fkm,  ', Data.L)
fprintf('\nUAV Flight Time:')
fprintf('%.2fs,  ', Data.t)
fprintf('\nUAV Flight Speed:')
fprintf('%.2fm/s,  ', Data.L./Data.t*1e3)
fprintf('\nTotal UAV Collisions: %d', Data.c)
fprintf('\nObjective Function Convergence: %.2f', Fitness_list(end))
% fprintf('\nα, β, δ Solution Numbers: %d,  %d,  %d', Alpha_no, Beta_no, Delta_no)
fprintf('\n\n')

end

%% Draw Spheres
function drawsphere(a, b, c, R, useSurf)
    % Centered at (a, b, c) with radius R
    if (nargin < 5)
        useSurf = false;
    end

    % Generate data
    [x, y, z] = sphere(20);

    % Adjust radius
    x = R * x; 
    y = R * y;
    z = R * z;

    % Adjust center
    x = x + a;
    y = y + b;
    z = z + c;
    
    if useSurf
        % Plot using surf
        axis equal;
        surf(x, y, z);
        hold on
    else
        % Plot using mesh
        axis equal;
        mesh(x, y, z);
        hold on
    end
end
