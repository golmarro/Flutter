
%% Collect controllers
clear

controllers = {};
descriptions = {};
colors = {'k','b','r','g','y'};

% open-loop
controllers{end+1} = [0 0 0 0];
descriptions{end+1} = 'open-loop';

[k descr] = classicsisocontroller();
controllers{end+1} = k;
descriptions{end+1} = descr;

[k descr] = lqgwingcontroller();
controllers{end+1} = k;
descriptions{end+1} = descr;

% [K descr] = HinfController();
% controllers{end+1} = K;
% descriptions{end+1} = descr;

%% Full analysis
f1 = figure; hold on;
f2 = figure; hold on;
for i = 2:length(controllers)
    analysis = LinearAnalysis(controllers{i});
    analysis.lineColor = colors{i};
    fprintf('Controller: %s\n', descriptions{i});
    analysis.fullAnalysis(f1, f2);
end
figure(f1)
legend(descriptions)
figure(f2)
legend(descriptions)

%% Compare response to pilot step input
f = axes; hold on;
for i = 1:length(controllers)
    analysis = LinearAnalysis(controllers{i});
    analysis.lineColor = colors{i};
    analysis.controlSignalAnalysis();
end
legend(descriptions)

%% Compare response to turbulence
f = figure; hold on;
for i = 1:length(controllers)
    analysis = LinearAnalysis(controllers{i});
    analysis.lineColor = colors{i};
    analysis.turbulenceAnalysis(1);
end
figure(f)
legend(descriptions)

%% Compare Hinf norm of delta_c output due to turbulence
f = figure; hold on;
for i = 2:length(controllers)
    analysis = LinearAnalysis(controllers{i});
    analysis.lineColor = colors{i};
    analysis.turbulenceHinfAnalysis();
end
figure(f)
legend(descriptions)
