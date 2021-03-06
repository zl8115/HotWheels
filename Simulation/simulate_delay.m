function [sim_results,bot,target] = simulate_delay(sim_vars,bot,target,delay)
    % Define the time scale
    t_end = sim_vars.t_end;
    t_step = sim_vars.t_step;
    t = [0:t_step:t_end];
    
    % Generate the target positions for the whole duration
    target.pos = gen_trajectories(sim_vars,target.speed,target.traj_option);

    for i = 1:length(t)
        % Getting Current Time Data
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

        % PID Controller State Update
        if delay == 0
            P = bot.PID(1)*(theta_e - bot.const_bearing); % Simple Pursuit if const_bearing = 0; else Constant Bearing    
        elseif i - delay <= 0
            P = 0;
        else
            P = bot.PID(1)*(bot.theta(4,i-delay) - bot.const_bearing);
        end
        I = 0; % Not needed, not coded...  
        if i - delay <= 2
            D = 0;        
        elseif delay == 0
            D = bot.PID(3)*(theta_r - bot.theta(3,i-delay-1))/t_step; % Parallel Navigation
        else
            D = bot.PID(3)*(bot.theta(3,i-delay) - bot.theta(3,i-delay-1))/t_step; % Parallel Navigation
        end
        
        % Theta_hdot calculation
        theta_hdot = P + I + D;

        % Theta State Update        
        bot.theta(:,i) = [theta_h; theta_hdot; theta_r; theta_e];
        
        % Relative Distance of Target from Bot
        dist(i) = norm(target_bot_diff);

        % Break Condition if Target is in Close Proximity to the Bot (Pursuer)
        if dist(i) < 5e-3
            %fprintf('Target Caught! \n');
            %fprintf('Time: %1.2f seconds \n',t(i));
            %fprintf('Bot Speed: %1.2f m/s \nTarget Speed: %1.2f m/s \n', bot.speed, target.speed);
            break
        end

        % Next State Updates
        x_vel = bot.speed*cosd(theta_h + theta_hdot*t_step);
        y_vel = bot.speed*sind(theta_h + theta_hdot*t_step);
        vel = [x_vel;y_vel]; % Velocity State Updates
        if i < length(t) % Ensures bot and target pos lengths remain the same
            bot.pos(:,i+1) = bot.pos(:,i) + t_step*vel; % + 0.0001*randn(2,1);
        end
    end
    
    sim_results.time = t(i); % Saves EndTime results
    sim_results.dist = dist; % Saves Distance Results
    sim_results.iter_num = i; % Saves Iteration Number
    sim_results.t_step = t_step; % Saves Time Steps
    
    target.pos = target.pos(:,1:size(bot.pos,2)); % Truncates TargetPos lengths
    
    % Ensures optimal axis for data plot
    y_est_bounds = [min([bot.pos(2,:);target.pos(2,:)],[],'all') max([bot.pos(2,:);target.pos(2,:)],[],'all')];
    x_est_bounds = [min([bot.pos(1,:);target.pos(1,:)],[],'all') max([bot.pos(1,:);target.pos(1,:)],[],'all')];
    sim_results.y_bounds = [min([y_est_bounds;sim_vars.y_bounds],[],'all') max([y_est_bounds;sim_vars.y_bounds],[],'all')];
    sim_results.x_bounds = [min([x_est_bounds;sim_vars.x_bounds],[],'all') max([x_est_bounds;sim_vars.x_bounds],[],'all')];
end