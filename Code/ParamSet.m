%% Agents Setting

agent(1) = Agent([-2, 2, 2],[2, -2, 2]);
agent(length(agent) + 1) = Agent([2, 2, 2],[-2, -2, 2]);
agent(length(agent) + 1) = Agent([2, -2, 2],[-2, 2, 2]);
agent(length(agent) + 1) = Agent([-2, -2, 2],[2, 2, 2]);

agent(length(agent) + 1) = Agent([0, 2, 2],[0, -2, 2]);
agent(length(agent) + 1) = Agent([0, -2, 2],[0, 2, 2]);
agent(length(agent) + 1) = Agent([2, 0, 2],[-2, 0, 2]);
agent(length(agent) + 1) = Agent([-2, 0, 2],[2, 0, 2]);



%% Obs Setting


%% Param Setting

% DMPC
param.h = 0.2;
param.N = 15;
param.kapa = 1;
param.rmin = 0.5;
param.f = 3;
param.posmin = [-2.5, -2.5, 0];
param.posmax = [2.5, 2.5, 5];
param.vmin = 1000*[-1, -1, -1];
param.vmax = 1000*[1, 1, 1];
param.amin = 1*[-1, -1, -1];
param.amax = 1*[1, 1, 1];
param.Q = 1*eye(3);
param.R = 0.2*eye(3);
param.S = 0.05*eye(3);
% param.R(3,3) = 1;
% param.S(3,3) = 1;

param.F = inv(diag([1,1,2]));
param.epsilonmax = 0.5;


% System
param.A = [eye(3), param.h*eye(3);
           zeros(3), eye(3)];
param.B = [param.h^2/2*eye(3);
           param.h*eye(3)];

% param.A = [eye(3), param.h*eye(3);
%            zeros(3), eye(3)];
% param.B = [zeros(3);
%             param.h*eye(3)];

%% Record Setting

for i = 1:length(agent)
    record.agentpos{i} = [agent(i).pos(1), agent(i).pos(2), agent(i).pos(3)];
    record.agentv{i} = [agent(i).v];
    record.agenta{i} = [agent(i).a];
    record.epsilon{i} = [];
end



