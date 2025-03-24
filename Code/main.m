clc;
close all;
clear;

%% Init
ParamSet;
plothandle = [];
figure();
hold on;
box on;
% set(gca,'FontSize', 10);
% axis([-3,3,-3,3]);
view([20,10,36]);
axis([-3,3,-3,3,0,5]);
xlabel('x(m)', 'Fontname', 'Times New Roman', 'fontsize', 10);
ylabel('y(m)', 'Fontname', 'Times New Roman', 'fontsize', 10);



%% Running
RunN = 0;
while 1

    RunN = RunN + 1

    % DMPC

    for i = 1:length(agent)
        i
        tic
        [agent(i).posN, agent(i).vN, agent(i).aN, agent(i).epsilon, time] = DMPC(agent(i), param);
        timerun = toc;
        timeall{RunN,i} = time;
        timeallrun{RunN,i} = timerun;
    end

    % update
    for i = 1:length(agent)
        agent(i) = agent(i).update();
        agent(i) = agent(i).checkarrive();
    end

    % GetNearAgent
    for i = 1:length(agent)
        agent(i).near_agent_index = GetNearAgent(i, agent, param.N, param.rmin, param.F, param.f);
    end

    % record

    for i = 1:length(agent)
        record.agentpos{i} = [record.agentpos{i}; agent(i).pos];
        record.agentv{i} = [record.agentv{i}; agent(i).v];
        record.agenta{i} = [record.agenta{i}; agent(i).a];
        % record.epsilon{i} = [record.epsilon{i}, i, agent(i).epsilon];
    end

    % draw trajectories
    delete(plothandle);
    plothandle = [];
    for i = 1:length(agent)
        % plothandle(length(plothandle)+1) = plot(agent(i).pos(1), agent(i).pos(2), 'ro-', 'LineWidth', 1.5, 'MarkerSize', 6);
        % plothandle(length(plothandle)+1) = plot(agent(i).posN(:,1), agent(i).posN(:,2), 'b--', 'LineWidth', 1.25);

        plothandle(length(plothandle)+1) = plot3(agent(i).pos(1), agent(i).pos(2), agent(i).pos(3), 'ro-', 'LineWidth', 1.5, 'MarkerSize', 6);
        plothandle(length(plothandle)+1) = plot3(agent(i).posN(:,1), agent(i).posN(:,2), agent(i).posN(:,3), 'b--', 'LineWidth', 1.25);
        
        if ~isempty(record.agentpos(1))
            % plothandle(length(plothandle)+1) = plot(record.agentpos{i}(:,1), record.agentpos{i}(:,2), 'r-', 'LineWidth', 1.25);
            plothandle(length(plothandle)+1) = plot3(record.agentpos{i}(:,1), record.agentpos{i}(:,2), record.agentpos{i}(:,3), 'r-', 'LineWidth', 1.25);
        end
    end
    F=getframe(gcf);
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);
    if RunN == 1
        imwrite(I,map,'soft_agents.gif','gif','Loopcount',inf,'DelayTime',0.2);
    else
        imwrite(I,map,'soft_agents.gif','gif','WriteMode','append','DelayTime',0.2);
    end


    % exit check
    flag_exit = 1;
    for i = 1:length(agent)
        if agent(i).flag_arrive == 0
            flag_exit = 0;
            break;
        end
    end
    if flag_exit == 1
        imwrite(I,map,'test1.gif','gif','Loopcount',inf,'DelayTime',0.1);
        break;
    end

end

save result.mat
plotresult;





