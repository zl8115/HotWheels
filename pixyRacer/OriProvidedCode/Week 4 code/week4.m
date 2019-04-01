clear;
close all;

motorSettings = [20; 30; 40; 50; 60; 70; 80; 90; 100];

leftWheelForwards = [0;84.1;135.2;185.3;235;283;330.3;376.5;416.9];
leftWheelBackwards = [18.9;88;148.5;198.2;244.8;292.1;334.8;373.4;386.0];
rightWheelForwards = [43.5;100.2;153.6;202.3;254.5;302.9;358.8;410.4;545.0];
rightWheelBackwards = [45.9;98.8;154.4;203.9;253.3;300.1;347;382.1;392.1];

varNames = {'motorSettings', 'leftWheelForwards', 'leftWheelBackwards', 'rightWheelForwards', 'rightWheelBackwards'};

for i = 1:5
   
    d = eval(['size(',varNames{i},')']);
    
    if d(1) == 1
        eval([varNames{i},'=transpose(',varNames{i},')']);
    elseif d(2) == 1
    else
        disp('you dun goofed');
        return
    end
    
end

if ~isequal(size(leftWheelForwards, 1), size(leftWheelBackwards, 1), size(rightWheelForwards, 1), size(rightWheelBackwards, 1))
    disp('you dun goofed');
    return
end

speeds = cat(2, leftWheelForwards, leftWheelBackwards, rightWheelForwards, rightWheelBackwards);

% truncate nonlinear sections of the plots
speeds = speeds(1:end, :);
motorSettings = motorSettings(1:end);

res = [ones(size(motorSettings, 1), 1), motorSettings]\speeds;

X = motorSettings(1):0.01:motorSettings(end);

figure;

for i = 1:4
    subplot(2, 2, i);
    Y = res(1, i) + res(2, i)*X;
    hold on;
    plot(motorSettings, eval(varNames{i+1}), 'xr');
    plot(X, Y);
    title(varNames{i+1});
    xlabel('motor speed');
    ylabel('turn rate');
    disp(varNames{i+1});
    disp(['a=',num2str(res(1, i))]);
    disp(['b=',num2str(res(2, i))]);
end


