function Tracks = Pops2Tracks(WolfPops, UAV)
%POPS2TRACKS Convert the population to tracks

SearchAgents = size(WolfPops.Pos, 1);    % Number of individuals in the population
UAVnum = UAV.num;                        % Number of UAVs
dim = UAV.PointDim;                      % Simulation dimension
v = WolfPops.Pos(:, end-UAVnum+1:end);   % Cooperative UAV velocities
P = WolfPops.Pos(:, 1:end-UAVnum);       % Cooperative UAV trajectories (xy)

Tracks = cell(SearchAgents, 1);
for agent = 1 : SearchAgents
    a.V = v(agent, :)';
    P_a = P(agent, :);
    a.P = cell(UAVnum, 1);
    for i = 1 : UAVnum
        PointNum = UAV.PointNum(i);   
        % For three-dimensional simulation, you need to modify this to PointNum*dim
        P_ai = P_a(1 : PointNum*dim);
        P_ai = reshape(P_ai, dim, PointNum);
        P_a = P_a(PointNum*dim+1 : end);
        a.P(i) = {P_ai};
    end
    Tracks(agent) = {a};
end

end
