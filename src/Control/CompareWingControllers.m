function CompareWingControllers(str)
    %% Collect controllers
    clear

    controllers = {};
    descriptions = {};
    ana = {};
    colors = {'k','b','r','g','y'};

    % open-loop
    controllers{end+1} = [0 0 0 0];
    descriptions{end+1} = 'open-loop';
    
    [K descr] = HinfController1();
    controllers{end+1} = K;
    descriptions{end+1} = descr;
    ana{end+1} = LinearAnalysis(K);
    
    [K descr] = HinfController2();
    controllers{end+1} = K;
    descriptions{end+1} = descr;
    ana{end+1} = LinearAnalysis(K);
    
    wing = WingFlutter;
    wing.U0 = wing.getFlutterSpeed;
    Gn = wing.getLinearModel;

    [K descr] = HinfController3(Gn,1);
    controllers{end+1} = K;
    descriptions{end+1} = 'Hinf 3.1';
    ana{end+1} = LinearAnalysis(K);
    %ana = LinearAnalysis(K);
    
    [K descr] = HinfController3(Gn,2);
    controllers{end+1} = K;
    descriptions{end+1} = 'Hinf 3.2';
    ana{end+1} = LinearAnalysis(K);
    
    wing.U0 = wing.getFlutterSpeed * 1.2;
    Gn = wing.getLinearModel;
    
    [K descr] = HinfController3(Gn,2);
    controllers{end+1} = K;
    descriptions{end+1} = 'Hinf 3.2 Gn dla 1.2*Vf';
    ana{end+1} = LinearAnalysis(K);

    [K descr] = ClassicSISOController();
    controllers{end+1} = K;
    descriptions{end+1} = descr;
    ana{end+1} = LinearAnalysis(K);

    % [k descr] = lqgwingcontroller();
    % controllers{end+1} = k;
    % descriptions{end+1} = descr;

    %% Flutter speed
    for i = 2:length(controllers)
        analysis = LinearAnalysis(controllers{i});
        fprintf('Controller: %s \t\t Vf: %f\n', descriptions{i}, analysis.getFlutterSpeed);
    end
    
    %% max delta analysis
    f = figure; hold on;
    for i = 2:length(controllers)
        analysis = LinearAnalysis(controllers{i});
        analysis.lineColor = colors{i};
        fprintf('Controller: %s\n', descriptions{i});
        analysis.maxDeltaAnalysis;
    end
    figure(f)
    legend(descriptions{2:end})
    
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
    legend(descriptions{2:end})
    figure(f2)
    legend(descriptions{2:end})

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
    legend(descriptions{2:end})
    
    %% Flutter speed versus fuel level
    f = figure; hold on;
    for i = 1:length(controllers)
        analysis = LinearAnalysis(controllers{i});
        analysis.lineColor = colors{i};
        analysis.wing.atmosphere.h = 5500;
        %analysis.flutterSpeedVersusFuel([0 0.5 1]);
        analysis.flutterSpeedVersusFuel([0.5]);
    end
    %%
    for i = 2:length(controllers)
        analysis = LinearAnalysis(controllers{i});
        analysis.lineColor = colors{i};
        analysis.wing.atmosphere.h = 5500;
        %analysis.flutterSpeedVersusFuel([0 0.5 1]);
        analysis.manouverSpeedVersusFuel([0 0.5 1]);
    end
    figure(f)
    legend(descriptions)
end