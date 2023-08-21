function IMG_AutoPlot(solution, UAV)
%IMG_AUTOPLOT Adaptive Drawing Function (the drawing is relatively ugly)
close all; 

% Solution
Tracks = solution.Tracks;                 % Tracks
Data = solution.Alpha_Data;               % Optimal track information
Fitness_list = solution.Fitness_list;     % Fitness curve
Alpha_no = solution.Alpha_no;             % α solution index
Beta_no = solution.Beta_no;               % β solution index
Delta_no = solution.Delta_no;             % γ solution index
agent_no = Alpha_no;                      % Index of the solution to be drawn

% Track diagram
if UAV.PointDim < 3

    %%%%%%%%% ———— 2D Simulation ———— %%%%%%%%%
    
    figure(1)
    for i = 1:UAV.num
        x = [UAV.S(i,1),Tracks{agent_no, 1}.P{i, 1}(1,:),UAV.G(i,1)];
        y = [UAV.S(i,2),Tracks{agent_no, 1}.P{i, 1}(2,:),UAV.G(i,2)];
        plot(x,y,LineWidth=2)   
        hold on
    end
    for i = 1:UAV.num
        plot(UAV.S(i,1),UAV.S(i,2),'ko',LineWidth=1,MarkerSize=9)
        hold on
        plot(UAV.G(i,1),UAV.G(i,2),'p',color='k',LineWidth=1,MarkerSize=10)
        hold on
    end
    for i = 1:size(UAV.Menace.radar,1)
        rectangle('Position',[UAV.Menace.radar(i,1)-UAV.Menace.radar(i,3),UAV.Menace.radar(i,2)-UAV.Menace.radar(i,3),2*UAV.Menace.radar(i,3),2*UAV.Menace.radar(i,3)],'Curvature',[1,1],'EdgeColor','k','FaceColor','g')
        hold on
    end
    for i = 1:size(UAV.Menace.other,1)
        rectangle('Position',[UAV.Menace.other(i,1)-UAV.Menace.other(i,3),UAV.Menace.other(i,2)-UAV.Menace.other(i,3),2*UAV.Menace.other(i,3),2*UAV.Menace.other(i,3)],'Curvature',[1,1],'EdgeColor','k','FaceColor','c')
        hold on
    end
    for i = 1:UAV.num
        leg_str{i} = ['Track',num2str(i)];  
    end
    leg_str{UAV.num+1} = 'Start';
    leg_str{UAV.num+2} = 'End';
    legend(leg_str)
    grid on
    axis equal
    dx = (max(UAV.limt.x(:,2))-min(UAV.limt.x(:,1)))*0.06;
    dy = (max(UAV.limt.y(:,2))-min(UAV.limt.y(:,1)))*0.06;
    xlim([min(UAV.limt.x(:,1))-dx,max(UAV.limt.x(:,2))+dx])
    ylim([min(UAV.limt.y(:,1))-dy,max(UAV.limt.y(:,2))+dy])
    xlabel('x(km)')
    ylabel('y(km)')
    title('Path Planning Diagram')
else

    %%%%%%%%% ———— 3D Simulation ———— %%%%%%%%%

    figure(1)
    for i = 1:UAV.num
        x = [UAV.S(i,1),Tracks{agent_no, 1}.P{i, 1}(1,:),UAV.G(i,1)];
        y = [UAV.S(i,2),Tracks{agent_no, 1}.P{i, 1}(2,:),UAV.G(i,2)];
        z = [UAV.S(i,3),Tracks{agent_no, 1}.P{i, 1}(3,:),UAV.G(i,3)];
        plot3(x,y,z,LineWidth=2)   
        hold on
    end
    for i = 1:UAV.num
        plot3(UAV.S(i,1),UAV.S(i,2),UAV.S(i,3),'ko',LineWidth=1.3,MarkerSize=12)
        hold on
        plot3(UAV.G(i,1),UAV.G(i,2),UAV.G(i,3),'p',color='k',LineWidth=1.3,MarkerSize=13)
        hold on
    end
    for i = 1:size(UAV.Menace.radar,1)
        drawsphere(UAV.Menace.radar(i,1),UAV.Menace.radar(i,2),UAV.Menace.radar(i,3),UAV.Menace.radar(i,4),true)
        hold on
    end
    for i = 1:size(UAV.Menace.other,1)
         drawsphere(UAV.Menace.other(i,1),UAV.Menace.other(i,2),UAV.Menace.other(i,3),UAV.Menace.other(i,4))
        hold on
    end
    for i = 1:UAV.num
        leg_str{i} = ['Track',num2str(i)];  
    end
    leg_str{UAV.num+1} = 'Start';
    leg_str{UAV.num+2} = 'End';
    legend(leg_str)
    grid on
    axis square
    %axis equal
    dx = (max(UAV.limt.x(:,2))-min(UAV.limt.x(:,1)))*0.06;
    dy = (max(UAV.limt.y(:,2))-min(UAV.limt.y(:,1)))*0.06;
    dz = (max(UAV.limt.z(:,2))-min(UAV.limt.z(:,1)))*0.06;
    xlim([min(UAV.limt.x(:,1))-dx,max(UAV.limt.x(:,2))+dx])
    ylim([min(UAV.limt.y(:,1))-dy,max(UAV.limt.y(:,2))+dy])
    zlim([min(UAV.limt.z(:,1))-dz,max(UAV.limt.z(:,2))+dz])
    xlabel('x(km)')
    ylabel('y(km)')
    zlabel('z(km)')
    title('Path Planning Diagram')
end

% Fitness
figure(2)
plot(Fitness_list,'k',LineWidth=1)
grid on
xlabel('iter')
ylabel('fitness')
title('Fitness Curve')

% Print information to the screen
fprintf('\nNumber of UAVs: %d', UAV.num)
fprintf('\nNumber of UAV navigation points: ')
fprintf('%d,  ', UAV.PointNum)
fprintf('\nUAV flight distance: ')
fprintf('%f,  ', Data.L)
fprintf('\nUAV flight time: ')
fprintf('%f,  ', Data.t)
fprintf('\nUAV flight speed: ')
fprintf('%f,  ',Data.L./Data.t*1e3)
fprintf('\nTotal number of collisions: %d', Data.c)
fprintf('\nOptimization convergence value: %f', Fitness_list(end))

end

function drawsphere(a,b,c,R,useSurf)
     
    if (nargin<5)
        useSurf = false;
    end
 
    [x,y,z] = sphere(20);
 
    x = R*x; 
    y = R*y;
    z = R*z;
 
    x = x+a;
    y = y+b;
    z = z+c;
    
    if useSurf 
        axis equal;
        surf(x,y,z);
        hold on
    else 
        axis equal;
        mesh(x,y,z);
        hold on
    end
end
