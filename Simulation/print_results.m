function [] = print_results(sim_results,bot,target)
    % Recreate time axis
    t = [0:sim_results.t_step:sim_results.time];

    figure;
    
    % Plot Trajectory of Target and Bot
    subplot(2,3,[1,2,4,5]);hold on;
    plot(bot.pos(1,end),bot.pos(2,end),'b','marker','o');
    plot(target.pos(1,end),target.pos(1,end),'r','marker','x');
    plot(bot.pos(1,:),bot.pos(2,:),'b');
    plot(target.pos(1,:),target.pos(2,:),'r');
    grid on;
    %ylim([min([bot.pos(2,:);target.pos(2,:)],[],'all') max([bot.pos(2,:);target.pos(2,:)],[],'all')]);
    %xlim([min([bot.pos(1,:);target.pos(1,:)],[],'all') max([bot.pos(1,:);target.pos(1,:)],[],'all')]);
    ylim(sim_results.y_bounds)
    xlim(sim_results.x_bounds)
    title('Trajectory Plot of Target and Bot')
    ylabel('Y-Coordinate');
    xlabel('X-Coordinate');
    legend('Bot(Pursuer)','Target');

    % Plot Distance of Target and Bot over Time
    subplot(2,3,3)
    plot(t,sim_results.dist);
    grid on;
    ylim([min(sim_results.dist) max(sim_results.dist)]);
    xlim([0 sim_results.time]);
    title('Distance Plot')
    ylabel('Distance')
    xlabel('Time(s)')

    % Plot Relative Error Angle of Target and Bot over Time
    subplot(2,3,6)
    plot(t,bot.theta(4,:));
    grid on;
    ylim([-180 180]);
    xlim([0 sim_results.time]);
    title('Error Angle Plot')
    ylabel('Angle(\circ)')
    xlabel('Time(s)')
end