close all, clear all
[sim_vars,bot,target] = init_vars();

t_end = sim_vars.t_end;
t_step = sim_vars.t_step;
t = [0:t_step:t_end]; 

for i = 1:3
    traj = gen_trajectories(sim_vars,target.speed,i);
    
    subplot(3,1,i); hold on;
    plot(traj(1,:),traj(2,:),'r');
    plot(traj(1,end),traj(2,end),'r','marker','x');
    grid on;
    title(['Trajectory Option ' num2str(i)])
    ylabel('Y-Coordinate');
    xlabel('X-Coordinate');
    ylim([-0.3 0.3])
    xlim([0 7])
end
    