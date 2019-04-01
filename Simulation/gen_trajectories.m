function positions = gen_trajectories(sim_vars,speed,option)
    % Recreate the time axis
    t_end = sim_vars.t_end;
    t_step = sim_vars.t_step;
    t = [0:t_step:t_end]; 
    
    % Generate different options
    if option == 2 % Sine Wave Trajectory (0.2Hz)
        x = [0:1:length(t)-1]*(speed*t_step) + 0.15;
        y = -0.3*sin(2*pi*0.2*t); % Sine Wave
    elseif option == 3 % Constant Speed Sine Wave Trajectory
        x = filter(1,[1 -1], abs((speed*t_step)*cos(2*pi*0.2*t))) + 0.15;
        y = filter(1,[1 -1], -(speed*t_step)*sin(2*pi*0.2*t));
    else % Linear Trajectory
        x = [0:1:length(t)-1]*(speed*t_step) + 0.15;
        y = zeros(1,length(t));
    end
    
    positions = [x;y];
end