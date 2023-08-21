%%% Gray Wolf Optimization for Trajectory Planning %%%
clc; close all;

%--- Algorithm Selection: 
algorithms = {'GWO'};

%--- Algorithm Parameter Settings
SearchAgents = 60;          % Number of search agents/wolf pack size/feasible solution count (>= 20)
Max_iter = 150;             % Maximum search iterations

%--- Cooperative UAV Setup
UAV = UAV_SetUp1;           % Set up in the UAV_SetUp.m file

solutions = {}; % To store the solutions of all algorithms

for idx = 1:length(algorithms)
    switch algorithms{idx}
        case 'GWO'
            solutions{end+1} = GWO(UAV, SearchAgents, Max_iter);
            IMG_AutoPlot(solutions{end}, UAV);
            
        case 'MP-GWO'
            solutions{end+1} = MP_GWO(UAV, SearchAgents, Max_iter);
            IMG_AutoPlot(solutions{end}, UAV);
            
        case 'CS-GWO'
            solutions{end+1} = CS_GWO(UAV, SearchAgents, Max_iter);
            IMG_AutoPlot(solutions{end}, UAV);

         case 'AGWO1'
            solutions{end+1} = AGWO1(UAV, SearchAgents, Max_iter);
            IMG_AutoPlot(solutions{end}, UAV);
        case 'AGWO2'
            solutions{end+1} = AGWO2(UAV, SearchAgents, Max_iter);
            IMG_AutoPlot(solutions{end}, UAV);
        otherwise
            disp('Invalid algorithm option');
    end
end

%--- If you want to manually plot the solutions after processing all algorithms
%for i = 1:length(solutions)
%    IMG_Plot(solutions{i}, UAV);
%end
