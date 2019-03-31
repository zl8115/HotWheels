close all, clear all

% Simulation Variables
t_end = 5; % 10 second simulation
t_step = 0.01; % Simulation Update Time Steps
x_bounds = [0,1];
y_bounds = [-0.25,0.25];
t = [0:t_step:t_end];
%const_bearing = 0;
const_bearing = 20;

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
bot.speed = 0.4; % 10cm/s or 0.1m/s
bot.x(1) = 0;
bot.y(1) = -0.15;
bot.theta_h(1) = 45;
bot.theta_hdot(1) = 0;
bot.theta_h_est(1) = 0;

for i = 1:length(t)
    % Relative Distance of Target from Bot
    dist(i) = norm([(target.y(i)-bot.y(i));(target.x(i)-bot.x(i))]);
    
    % Break Condition if Target is in Close Proximity to the Bot (Pursuer)
    if norm([(target.y(i)-bot.y(i));(target.x(i)-bot.x(i))]) < 5e-3
        fprintf('Target Caught! \n');
        fprintf('Time: %1.2f seconds \n',t(i));
        fprintf('Bot Speed: %1.2f m/s \nTarget Speed: %1.2f m/s \n', bot.speed, target.speed);
        break
    end
    
    % Relative Angle of Target from Bot (Pursuer)
    theta_r(i) = atan2d((target.y(i)-bot.y(i)),(target.x(i)-bot.x(i)));    
    if theta_r(i) <= 0 % Ensures angle is within [0,360]
        theta_r(i) = 360 + theta_r(i);
    end
    
    % Relative Error Angle of Target from Bot
    theta_e(i) = theta_r(i) - bot.theta_h(i);    
    if abs(theta_e(i)) > 180 % Ensures angle is within [-180,180]
        theta_e(i) = -sign(theta_e(i))*(360 - abs(theta_e(i)));        
    end
    
    % Estimation Shit
    bot.theta_h_est(i+1) = sum(bot.theta_hdot*t_step);
    bot.theta_r_est(i) = bot.theta_h_est(i) - theta_e(i);
    if i == 1
        bot.theta_rdot_est(i) = 0;
    else
        bot.theta_rdot_est(i) = (bot.theta_r_est(i)-bot.theta_r_est(i-1))/t_step;
    end
    
    % PID Controller State Update    
    P(i) = Kp*(theta_e(i)-const_bearing); % Simple Pursuit if const_bearing = 0; else Constant Bearing    
    I(i) = 0; % Not needed, not coded...  
    if i == 1
        D(i) = 0;
    else        
        %D(i) = Kd*(theta_r(i)-theta_r(i-1))/t_step; % Parallel Navigation
        D(i) = Kd*bot.theta_rdot_est(i);
    end
    bot.theta_hdot(i) = P(i) + I(i) + D(i);
    
    % Velocity State Updates
    x_vel = bot.speed*cosd(bot.theta_h(i) + bot.theta_hdot(i)*t_step);
    y_vel = bot.speed*sind(bot.theta_h(i) + bot.theta_hdot(i)*t_step);
    
    % Next State Updates
    bot.x(i+1) = bot.x(i) + t_step*x_vel; % + 0.0001*randn();
    bot.y(i+1) = bot.y(i) + t_step*y_vel; % + 0.0001*randn();
    %bot.theta_h(i+1) = bot.theta_h(i) + t_step*bot.theta_hdot(i);
    bot.theta_h(i+1) = atan2d(bot.y(i+1)-bot.y(i),bot.x(i+1)-bot.x(i));    
    if bot.theta_h(i+1) <= 0 % Ensures angle is within [0,360]
        bot.theta_h(i+1) = 360 + bot.theta_h(i+1);
    end
    
    %{
    % State Plot Update
    clf;
    subplot(2,1,1)
    hold on;
    % Bot Plot Update
    plot(bot.x(i),bot.y(i),'b','marker','o');
    plot(bot.x(1:i),bot.y(1:i),'b');
    % Target Plot Update
    plot(target.x(i),target.y(i),'r','marker','x');
    plot(target.x(1:i),target.y(1:i),'r');
    % plot([target.x(i); bot.x(i)],[target.y(i); bot.y(i)] ,'c'); % Target and Bot Line Intercept Plot
    grid on;
    ylim(y_bounds);
    xlim(x_bounds);   
    %}
    pause(t_step)
    %}
end

figure; 
sgtitle(sprintf("PID=[%1.2f,%1.2f,%1.2f]; Const\\_Bearing=%i; BotSpeed=%1.2f; TargetSpeed=%1.2f", Kp, Ki, Kd, const_bearing, bot.speed, target.speed))
subplot(2,3,[1,2,4,5]);hold on;
plot(bot.x(i),bot.y(i),'b','marker','o');
plot(target.x(i),target.y(i),'r','marker','x');
plot(bot.x,bot.y,'b');
plot(target.x(1:i),target.y(1:i),'r');
grid on;
ylim(y_bounds);
xlim(x_bounds);
title('Trajectory Plot of Target and Bot')
ylabel('Y-Coordinate');
xlabel('X-Coordinate');
legend('Bot(Pursuer)','Target');

subplot(2,3,3)
plot(t(1:i),dist);
grid on;
ylim([0 max(y_bounds)-min(y_bounds)])
xlim([0 t_end]);
title('Distance Plot')
ylabel('Distance')
xlabel('Time(s)')

subplot(2,3,6)
plot(t(1:length(theta_e)),theta_e);
grid on;
ylim([-180 180]);
xlim([0 t_end]);
title('Error Angle Plot')
ylabel('Angle(\circ)')
xlabel('Time(s)')