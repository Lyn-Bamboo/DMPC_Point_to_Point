%% draw trajectories
figure;
hold on;
for i = 1:length(record.agentpos)
    plot3(record.agentpos{1,i}(:,1), record.agentpos{1,i}(:,2), record.agentpos{1,i}(:,3),'LineWidth',2);
end
view([20,10,36]);
axis([-3,3,-3,3,0,5]);
box on;
xlabel('x(m)', 'Fontname', 'Times New Roman', 'fontsize', 10);
ylabel('y(m)', 'Fontname', 'Times New Roman', 'fontsize', 10);
zlabel('z(m)', 'Fontname', 'Times New Roman', 'fontsize', 10);


%% solver time
figure;
boxplot(cell2mat(timeall));
xlabel('subsystem');
ylabel('comp time/s')


%% run time each step
figure
boxplot(cell2mat(timeallrun));
xlabel('subsystem'); 
ylabel('yalmip runtime/s')