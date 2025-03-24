function [posN, vN, aN, epsilon_ij, time] = DMPC(agent_i, param)

%% Init


N = param.N;
kapa = param.kapa;
rmin = param.rmin;
f = param.f;
posmin = param.posmin;
posmax = param.posmax;
vmin = param.vmin;
vmax = param.vmax;
amin = param.amin;
amax = param.amax;
epsilonmax = param.epsilonmax;

Q = param.Q;
R = param.R;
S = param.S;
F = param.F;


A = param.A;
B = param.B;


yalmip('clear');
Con = [];
J = 0;
Je = 0;
Ju = 0;
Jj = 0;
Jc = 0;

posN = sdpvar(N, length(agent_i.pos), 'full');
vN = sdpvar(N, length(agent_i.v), 'full');
aN = sdpvar(N, length(agent_i.a), 'full');
epsilon_ij = sdpvar(N, length(agent_i.near_agent_index), 'full');



%% Constraints

% x(k+1) = A*x(k) + B*u(k)
Con = [Con, [posN(1,:), vN(1,:)]' == (A*[agent_i.pos, agent_i.v]' + B*agent_i.a')];
for j = 1:N-1
    Con = [Con, [posN(j+1,:), vN(j+1,:)]' == A*[posN(j,:), vN(j,:)]' + B*aN(j,:)'];


% |pos| < pmax, |u| < umax
    Con = [Con, posmin <= posN(j,:) <= posmax];
    Con = [Con, vmin <= vN(j,:) <= vmax];
    Con = [Con, amin <= aN(j,:) <= amax];
end
Con = [Con, posmin <= posN(N,:) <= posmax];
Con = [Con, amin <= aN(N,:) <= amax];


% ||posi-posj|| > rmin

% % hard constraints
% if ~isempty(agent_i.near_agent_index)
%     for j = 1:length(agent_i.near_agent_index)
%         agent_j_posN = agent_i.near_agent_index{j};
%         for k = 1:N
%             ksi = norm(F*(agent_i.posN(k,:) - agent_j_posN(k,:))');
%             if ksi < f*rmin
%                 % k
%                 % ksi
%                 v = F*(agent_i.posN(k,:) - agent_j_posN(k,:))';
%                 aa = rmin*ksi -ksi^2 +agent_i.posN(k,:)*v;
%                 Con = [Con, posN(k,:)*v >= rmin*ksi -ksi^2 +agent_i.posN(k,:)*v];
%             end
%         end
%     end
% end

% soft constraints
if ~isempty(agent_i.near_agent_index)
    for j = 1:length(agent_i.near_agent_index)
        agent_j_posN = agent_i.near_agent_index{j};
        for k = 1:N
            Con = [Con, -epsilonmax <= epsilon_ij(k,j) <= 0];
            ksi = norm(F*(agent_i.posN(k,:) - agent_j_posN(k,:))');
            if ksi < f*rmin
                % k
                % ksi
                v = F*(agent_i.posN(k,:) - agent_j_posN(k,:))';
                aa = rmin*ksi -ksi^2 +agent_i.posN(k,:)*v;
                Con = [Con, posN(k,:)*v - epsilon_ij(k,j)*ksi >= rmin*ksi - ksi^2 + agent_i.posN(k,:)*v];
            end
        end
    end
end




%% Objective Function

% Je = ||posN(N-kapa:N)-posgoal||Q
for j = 0:kapa
    Je = Je + (posN(N-j,:) - agent_i.goal)*Q*(posN(N-j,:) - agent_i.goal)';
end


% Ju = ||a||R
for j = 1:N
    Ju = Ju + aN(j,:)*R*(aN(j,:))';
end


% Jj = ||a(j+1) - a(j)||S
Jj = Jj + (aN(1,:) - agent_i.a)*S*(aN(1,:) - agent_i.a)';
for j = 1:N-1
    Jj = Jj + (aN(j+1,:) - aN(j,:))*S*(aN(j+1,:) - aN(j,:))';
end


% J
% hard constraints
% J = Je + Ju + Jj;


% soft constraints
% Jc = ||epsilon||H
H = 150*eye(length(agent_i.near_agent_index));
for k = 1:N
    Jc = Jc + epsilon_ij(k,:)*H*epsilon_ij(k,:)'; 
end

% J
J = Je + Ju + Jj + Jc;




%% Solve

ops = sdpsettings('solver','osqp','osqp.alpha',1.6);
% ops = sdpsettings('solver','osqp');
result = optimize(Con, J, ops);
% result = optimize(Con, J);
if ~(isequal(result.info,'Successfully solved (Successfully solved (QUADPROG))') ||...
     isequal(result.info,'Successfully solved (fmincon-standard)') || ...
     isequal(result.info,'Successfully solved (Successfully solved (OSQP))'))
    warning('TIME OUT!');
    posN = [agent_i.posN(2:end,:); agent_i.posN(end,:)];
    vN = [agent_i.vN(2:end,:); agent_i.vN(end,:)];
    aN = [agent_i.aN(2:end,:); agent_i.aN(end,:)];
    epsilon_ij = [];
else
    posN = value(posN);
    vN = value(vN);
    aN = value(aN);
    epsilon_ij = value(epsilon_ij);
end
time =  result.solvertime;


end
