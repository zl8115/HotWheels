clear all;

% Default Settings Simulation
[sim_vars,bot,target] = init_vars();

% Generating 3 bots for different pursuits
% Simple Pursuit
bot1 = bot;
% Constant Bearing Pursuit
bot2 = bot;
bot2.const_bearing = 20;
% Parallel Navigation Pursuit
bot3 = bot;
bot3.PID = [0;0;5];

delays = [0:2:20];

for i = 1:length(delays)
    
    [sim_results1,bot_results1,target_results1] = simulate_delay(sim_vars, bot1, target, delays(i));
    save1{i} = struct('sim_results',sim_results1,'bot',bot_results1,'target',target_results1);
    
    [sim_results2,bot_results2,target_results2] = simulate_delay(sim_vars, bot2, target, delays(i));
    save2{i} = struct('sim_results',sim_results2,'bot',bot_results2,'target',target_results2);
    
    [sim_results3,bot_results3,target_results3] = simulate_delay(sim_vars, bot3, target, delays(i));
    save3{i} = struct('sim_results',sim_results3,'bot',bot_results3,'target',target_results3);
    
    fprintf("Delay: %1.2f; EndTime1: %1.3f; EndTime2: %1.3f; EndTime3: %1.3f; \n",delays(i)*sim_vars.t_step, sim_results1.time, sim_results2.time, sim_results3.time);

    results(1,i) = sim_results1.time;
    results(2,i) = sim_results2.time;
    results(3,i) = sim_results3.time;
end