function [sim_vars,bot,target] = init_vars()
    sim_vars = struct('t_end',30, 't_step',0.01, 'x_bounds', [0,1], 'y_bounds', [-0.25,0.25]);
    
    target.speed = 0.2;
    target.traj_option = 2;

    bot.speed = 0.4;
    bot.pos(:,1) = [0;-0.15]; %[x; y];
    bot.theta(:,1) = [90;0;0;0]; %[theta_h; theta_hdot; theta_r; theta_e]
    bot.PID = [5;0;0]; %[Kp; Ki: Kd]
    bot.const_bearing = 0;
end