clear all;

% Default Settings Simulation
[sim_vars,bot,target] = init_vars();

[sim_results,bot,target] = simulate(sim_vars, bot, target);
print_results(sim_results,bot,target)

fprintf("Default Settings; EndTime: %1.2f \n",sim_results.time);

%% Different Trajectories
[sim_vars,bot,target] = init_vars();

traj_options = [1,2,3];

for i = 1:length(traj_options)
    target.traj_option = traj_options(i);
    
    [sim_results,bot_results,target_results] = simulate(sim_vars, bot, target);
    save1{i} = struct('sim_results',sim_results,'bot',bot_results,'target',target_results);
    fprintf("TargetTrajectoryOption: %i; EndTime: %1.2f \n",traj_options(i),sim_results.time);

    traj_options_results(i) = sim_results.time;
end

%{
i = 2;

bot = save1{i}.bot;
target = save1{i}.target;
sim_results = save1{i}.sim_results;
print_results(sim_results,bot,target);
%}

%% Bot Speed vs Target Speed Section
[sim_vars,bot,target] = init_vars();

bot_speeds = [0.2:0.1:0.8];
target_speeds = [0.1:0.1:0.8];

for i = 1:length(target_speeds)
    for j = 1:length(bot_speeds)
        target.speed = target_speeds(i);
        bot.speed = bot_speeds(j);
        
        [sim_results,bot_results,target_results] = simulate(sim_vars, bot, target);
        save2{i,j} = struct('sim_results',sim_results,'bot',bot_results,'target',target_results);
        fprintf("TargetSpeed: %1.2f; BotSpeed: %1.2f; EndTime: %1.2f \n",target_speeds(i),bot_speeds(j),sim_results.time);
        
        varspeeds_time_results(i,j) = sim_results.time;
    end
end

%{
i = 4;
j = 6;

bot = save2{i,j}.bot;
target = save2{i,j}.target;
sim_results = save2{i,j}.sim_results;
print_results(sim_results,bot,target)
%}

%% Kp Gain vs Constant Bearing
[sim_vars,bot,target] = init_vars();

Kp_vals = [0.5,1,5,10,25,100];
const_bearing_vals = [0,10,20,45,90];

for i = 1:length(Kp_vals)
    for j = 1:length(const_bearing_vals)
        bot.PID = [Kp_vals(i);0;0];
        bot.const_bearing = const_bearing_vals(j);        
        
        [sim_results,bot_results,target_results] = simulate(sim_vars, bot, target);
        save3{i,j} = struct('sim_results',sim_results,'bot',bot_results,'target',target_results);
        fprintf("KpVal: %1.2f; ConstantBearing: %1.2f; EndTime: %1.2f \n",Kp_vals(i),const_bearing_vals(j),sim_results.time);
        
        Kp_bearing_results(i,j) = sim_results.time;
    end
end

%{
i = 5;
j = 5;

bot = save3{i,j}.bot;
target = save3{i,j}.target;
sim_results = save3{i,j}.sim_results;
print_results(sim_results,bot,target)
%}

%% Kp Gain vs Bot Speed
[sim_vars,bot,target] = init_vars();

Kp_vals = [0.5,1,5,10,25,100];
bot_speeds = [0.2:0.1:0.8];

for i = 1:length(Kp_vals)
    for j = 1:length(bot_speeds)
        bot.PID = [Kp_vals(i);0;0];
        bot.speed = bot_speeds(j);
        
        [sim_results,bot_results,target_results] = simulate(sim_vars, bot, target);
        save4{i,j} = struct('sim_results',sim_results,'bot',bot_results,'target',target_results);
        fprintf("KpVal: %1.2f; BotSpeed: %1.2f; EndTime: %1.2f \n",Kp_vals(i),bot_speeds(j),sim_results.time);
        
        Kp_botspeed_results(i,j) = sim_results.time;
    end
end

%{
i = 5;
j = 5;

bot = save4{i,j}.bot;
target = save4{i,j}.target;
sim_results = save4{i,j}.sim_results;
print_results(sim_results,bot,target)
%}

%% Kp Gain vs Target Speed
[sim_vars,bot,target] = init_vars();

Kp_vals = [0.5,1,5,10,25,100];
target_speeds = [0.1:0.1:0.8];

for i = 1:length(Kp_vals)
    for j = 1:length(target_speeds)
        bot.PID = [Kp_vals(i);0;0];
        target.speed = target_speeds(j);
        
        [sim_results,bot_results,target_results] = simulate(sim_vars, bot, target);
        save5{i,j} = struct('sim_results',sim_results,'bot',bot_results,'target',target_results);
        fprintf("KpVal: %1.2f; TargetSpeed: %1.2f; EndTime: %1.2f \n",Kp_vals(i),target_speeds(j),sim_results.time);
        
        Kp_targetspeed_results(i,j) = sim_results.time;
    end
end

%{
i = 5;
j = 5;

bot = save5{i,j}.bot;
target = save5{i,j}.target;
sim_results = save5{i,j}.sim_results;
print_results(sim_results,bot,target)
%}

%% Heatmap Tuning Curve?

figure;
sgtitle(['Tuning Maps for Trajectory Option ' num2str(target.traj_option)])
subplot(2,2,1);
%heatmap(bot_speeds,target_speeds,varspeeds_time_results,'Colormap',hot)
heatmap(bot_speeds,target_speeds,varspeeds_time_results)
title('BotSpeed vs TargetSpeed')
xlabel('Bot Speed')
ylabel('Target Speed')

subplot(2,2,2);
heatmap(const_bearing_vals,Kp_vals,Kp_bearing_results)
title('AngularConstant vs P-Gain(K_p)')
xlabel('Angular Constant')
ylabel('P-Gain (K_p)')

subplot(2,2,3);
heatmap(bot_speeds,Kp_vals,Kp_botspeed_results)
title('BotSpeed vs P-Gain(K_p)')
xlabel('BotSpeed')
ylabel('P-Gain (K_p)')

subplot(2,2,4);
heatmap(target_speeds,Kp_vals,Kp_targetspeed_results)
title('TargetSpeed vs P-Gain(K_p)')
xlabel('TargetSpeed')
ylabel('P-Gain (K_p)')

%% Print Results

[row,col] = find(varspeeds_time_results == min(varspeeds_time_results,[],'all'));
fprintf('Opt BotSpeed vs TargetSpeed: BotSpeed=%1.2f; TargetSpeed=%1.2f \n', bot_speeds(row),target_speeds(col));

[row,col] = find(Kp_bearing_results == min(Kp_bearing_results,[],'all'));
fprintf('Opt Kp vs ConstBearing: Kp=%1.2f; ConstBearing=%1.2f \n', Kp_vals(row),const_bearing_vals(col));

[row,col] = find(Kp_botspeed_results == min(Kp_botspeed_results,[],'all'));
fprintf('Opt Kp vs BotSpeed: Kp=%1.2f; BotSpeed=%1.2f \n', Kp_vals(row),bot_speeds(col));

[row,col] = find(Kp_targetspeed_results == min(Kp_targetspeed_results,[],'all'));
fprintf('Opt Kp vs TargetSpeed: Kp=%1.2f; TargetSpeed=%1.2f \n', Kp_vals(row),target_speeds(col));
%%
%{
figure; 
hold on;
plot(save{i,j}.bot.pos(1,end),save{i,j}.bot.pos(2,end),'b','marker','o');
plot(save{i,j}.target.pos(1,end),save{i,j}.target.pos(2,end),'r','marker','x');
plot(save{i,j}.bot.pos(1,:),save{i,j}.bot.pos(2,:),'b');
plot(save{i,j}.target.pos(1,:),save{i,j}.target.pos(2,:),'r');
grid on;
ylim(sim_vars.y_bounds);
xlim(sim_vars.x_bounds);
title('Trajectory Plot of Target and Bot')
ylabel('Y-Coordinate');
xlabel('X-Coordinate');
legend('Bot(Pursuer)','Target');
%}

%{
function positions = gen_trajectories(t,speed,option)
    t_step = mean(diff(t));
    if option == 2 % Sine Wave Trajectory (0.2Hz)
        x = [0:1:length(t)-1]*(speed*t_step) + 0.15;
        y = -sin(2*pi*0.2*t); % Sine Wave
    elseif option == 3 % Constant Speed Sine Wave Trajectory
        x = filter(1,[1 -1], abs((speed*t_step)*cos(2*pi*0.2*t))) + 0.15;
        y = filter(1,[1 -1], -(speed*t_step)*sin(2*pi*0.2*t));
    else
        x = [0:1:length(t)-1]*(speed*t_step) + 0.15;
        y = zeros(1,length(t));
    end
    
    positions = [x;y];
end

function [sim_results,bot,target] = simulate(sim_vars,bot,target)
    t_end = sim_vars.t_end;
    t_step = sim_vars.t_step;
    t = [0:t_step:t_end];    
    target.pos = gen_trajectories(t,target.speed,target.traj_option);

    for i = 1:length(t)
        target_pos_now = target.pos(:,i);
        bot_pos_now = bot.pos(:,i);
        target_bot_diff = target_pos_now - bot_pos_now;
        
        if i == 1
            theta_h = bot.theta(1,i);
        else
            bot_prev_vel = bot.pos(:,i) - bot.pos(:,i-1); 
            theta_h = atan2d(bot_prev_vel(2),bot_prev_vel(1));
            if theta_h <= 0 % Ensures angle is within [0,360]
                theta_h = 360 + theta_h;
            end
        end
        
        % Relative Angle of Target from Bot (Pursuer)
        theta_r = atan2d(target_bot_diff(2),target_bot_diff(1));
        if theta_r <= 0 % Ensures angle is within [0,360]
            theta_r = 360 + theta_r;
        end

        % Relative Error Angle of Target from Bot
        theta_e = theta_r - theta_h;
        if abs(theta_e) > 180 % Ensures angle is within [-180,180]
            theta_e = -sign(theta_e)*(360 - abs(theta_e));        
        end

        % Relative Distance of Target from Bot
        dist(i) = norm(target_bot_diff);

        % Break Condition if Target is in Close Proximity to the Bot (Pursuer)
        if dist(i) < 5e-3
            %fprintf('Target Caught! \n');
            %fprintf('Time: %1.2f seconds \n',t(i));
            %fprintf('Bot Speed: %1.2f m/s \nTarget Speed: %1.2f m/s \n', bot.speed, target.speed);
            break
        end

        % PID Controller State Update    
        P = bot.PID(1)*(theta_e - bot.const_bearing); % Simple Pursuit if const_bearing = 0; else Constant Bearing    
        I = 0; % Not needed, not coded...  
        if i == 1
            D = 0;
        else        
            D = bot.PID(3)*(theta_r - bot.theta(3,i-1))/t_step; % Parallel Navigation            
        end
        theta_hdot = P + I + D;

        % Velocity State Updates
        x_vel = bot.speed*cosd(theta_h + theta_hdot*t_step);
        y_vel = bot.speed*sind(theta_h + theta_hdot*t_step);
        vel = [x_vel;y_vel];

        % Next State Updates
        if i < length(t)
            bot.pos(:,i+1) = bot.pos(:,i) + t_step*vel; % + 0.0001*randn(2,1);
        end
        bot.theta(:,i) = [theta_h; theta_hdot; theta_r; theta_e];
    end
    
    sim_results.time = t(i);
    sim_results.dist = dist;
    sim_results.iter_num = i;
    sim_results.t_step = t_step;
    
    target.pos = target.pos(:,1:size(bot.pos,2));
    %sim_results.y_bounds = [min([bot.pos(2,:);target.pos(2,:)]) max([bot.pos(2,:);target.pos(2,:)])];
    %sim_results.x_bounds = [min([bot.pos(1,:);target.pos(1,:)]) max([bot.pos(1,:);target.pos(1,:)])];
end

function [] = print_results(sim_results,bot,target)
    t = [0:sim_results.t_step:sim_results.time];

    figure;
    subplot(2,3,[1,2,4,5]);hold on;
    plot(bot.pos(1,end),bot.pos(2,end),'b','marker','o');
    plot(target.pos(1,end),target.pos(1,end),'r','marker','x');
    plot(bot.pos(1,:),bot.pos(2,:),'b');
    plot(target.pos(1,:),target.pos(2,:),'r');
    grid on;
    ylim([min([bot.pos(2,:);target.pos(2,:)],[],'all') max([bot.pos(2,:);target.pos(2,:)],[],'all')]);
    xlim([min([bot.pos(1,:);target.pos(1,:)],[],'all') max([bot.pos(1,:);target.pos(1,:)],[],'all')]);
    title('Trajectory Plot of Target and Bot')
    ylabel('Y-Coordinate');
    xlabel('X-Coordinate');
    legend('Bot(Pursuer)','Target');

    subplot(2,3,3)
    plot(t,sim_results.dist);
    grid on;
    ylim([min(sim_results.dist) max(sim_results.dist)]);
    xlim([0 sim_results.time]);
    title('Distance Plot')
    ylabel('Distance')
    xlabel('Time(s)')

    subplot(2,3,6)
    plot(t,bot.theta(4,:));
    grid on;
    ylim([-180 180]);
    xlim([0 sim_results.time]);
    title('Error Angle Plot')
    ylabel('Angle(\circ)')
    xlabel('Time(s)')
end
%}