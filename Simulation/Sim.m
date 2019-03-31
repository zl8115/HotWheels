close all, clear all

% Simulation Variables
t_end = 5; % 10 second simulation
t_step = 0.01; % Simulation Update Time Steps
x_bounds = [0,1];
y_bounds = [-0.20,0.20];
t = [0:t_step:t_end];
%const_bearing = 20*pi/180;
const_bearing = 0;

% PID Tunings
Kp = 1;
Ki = 0;
Kd = 0; % 50 works well

% Target Trajectory Tunings
move_freq = 0.5;
move_amp = 0.1;
target.speed = 0.2;

% Linear Trajectory
%target.x = [0:1:length(t)-1]*(target.speed*t_step) + 0.15;
%target.y = zeros(length(t),1);

% Sine Wave Trajectory
%target.x = [0:1:length(t)-1]*(target.speed*t_step) + 0.15;
%target.y = -move_amp*sin(2*pi*move_freq*t); % Sine Wave

% Constant Speed Sine Wave Trajectory
target.x = filter(1,[1 -1], abs((target.speed*t_step)*cos(2*pi*move_freq*t))) + 0.15;
target.y = filter(1,[1 -1], -(target.speed*t_step)*sin(2*pi*move_freq*t));

% Bot (Pursuer) Tunings
bot.x(1) = 0;
bot.y(1) = -0.15;
bot.theta_h(1) = pi/4;
bot.theta_h2(1) = pi/2;
bot.theta_hdot(1) = 0;
bot.speed = 0.4; % 10cm/s or 0.1m/s
bot.theta_h_est(1) = 0;

for i = 1:length(t)
    % Break Condition if Target is in Close Proximity to the Bot (Pursuer)
    if norm([(target.y(i)-bot.y(i));(target.x(i)-bot.x(i))]) < 2e-3
        fprintf('Target Caught! \n');
        fprintf('Time: %1.2f seconds \n',t(i));
        fprintf('Bot Speed: %1.2f m/s \nTarget Speed: %1.2f m/s \n', bot.speed, target.speed);
        break
    end
    
    % Relative Angle of Target from Bot (Pursuer)
    theta_r(i) = atan2((target.y(i)-bot.y(i)),(target.x(i)-bot.x(i)));
    %{
    if theta_r(i) < 0
        theta_r(i) = pi - theta_r(i);
    end
    %}
    
    % Relative Distance of Target from Bot
    dist(i) = norm([(target.y(i)-bot.y(i));(target.x(i)-bot.x(i))]);
    
    % Relative Error Angle of Target from Bot
    theta_e(i) = theta_r(i) - bot.theta_h(i);
    if abs(theta_e(i)) > pi
        theta_e(i) = 
    
    % Estimation Shit
    bot.theta_h_est(i+1) = sum(bot.theta_hdot*t_step);
    bot.theta_r_est(i) = bot.theta_h_est(i) - theta_e(i);
    if i == 1
        bot.theta_rdot_est(i) = 0;
    else
        bot.theta_rdot_est(i) = (bot.theta_r_est(i)-bot.theta_r_est(i-1))/t_step;
    end
    
    % PID Controller State Update    
    P(i) = Kp*(theta_e(i)); % Simple Pursuit
    %P(i) = Kp*(theta_r(i)-bot.theta_h(i)-const_bearing); % Constant Bearing
    I(i) = 0; % Not needed, not coded...  
    if i == 1
        D(i) = 0;
    else        
        %D(i) = Kd*(theta_r(i)-theta_r(i-1))/t_step; % Parallel Navigation
        D(i) = Kd*bot.theta_rdot_est(i);
    end
    bot.theta_hdot(i) = P(i) + I(i) + D(i);
    
    % Velocity State Updates
    x_vel = bot.speed*cos(bot.theta_h(i) + bot.theta_hdot(i)*t_step);
    y_vel = bot.speed*sin(bot.theta_h(i) + bot.theta_hdot(i)*t_step);
    
    % Next State Updates
    bot.x(i+1) = bot.x(i) + t_step*x_vel; % + 0.0001*randn();
    bot.y(i+1) = bot.y(i) + t_step*y_vel; % + 0.0001*randn();
    %bot.theta_h(i+1) = bot.theta_h(i) + t_step*bot.theta_hdot(i);
    bot.theta_h(i+1) = atan2(bot.y(i+1)-bot.y(i),bot.x(i+1)-bot.x(i));
    %{
    if bot.theta_h(i+1) < 0
        bot.theta_h(i+1) = pi - bot.theta_h(i+1);
    end
    %}
    % Plot Update
    clf;
    subplot(2,1,1)
    hold on;
    % Bot Plot Update
    plot(bot.x(i),bot.y(i),'b','marker','o');
    plot(bot.x(1:i),bot.y(1:i),'b');
    % Target Plot Update
    plot(target.x(i),target.y(i),'r','marker','x');
    plot(target.x(1:i),target.y(1:i),'r');
    
    %plot([target.x(i); bot.x(i)],[target.y(i); bot.y(i)] ,'c');
    grid on;
    ylim(y_bounds);
    xlim(x_bounds);
    
    subplot(2,1,2)
    plot(dist)
    xlim([0 length(t)-1])
    ylim([0 max(y_bounds)-min(y_bounds)])
    
    pause(t_step)
end

%figure; plot(theta_r)
%ylim([-pi, pi]);