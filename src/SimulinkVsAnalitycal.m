path(path, 'WingFlutter');
path(path, 'Simulink');

%%
clear
close_system('WingSim', 0);
wingParams = WingParams();
wing = WingFlutter(wingParams);
simRunner = SimRunner('simRunner', wing);
wing.simStyle = 'r--';

%% no aero (structural)

figure
wing.isGravity = 'on';
wing.AeroForces = 'off';
wingParams.Xac_p = 0;
%wingParams.Xcg_p = 0;
%wingParams.Yac_p = 1/7;
%wingParams.Ycg_p = 1/7;

simRunner.sim(4);
wing.sim(4);
legend('Simulink', 'Analytical');

%% No gravity, no aero, initial state

figure
wing.isGravity = 'off';
wing.AeroForces = 'off';
wingParams.Xcg_p = 0;

state = [0 3*pi/180 0 0];
simRunner.sim(4, state);
wing.sim(4, state);
legend('Simulink', 'Analytical');

%% All
figure
wing.isGravity = 'on';
wing.AeroForces = 'on';

simRunner.sim(4);
wing.sim(4);
legend('Simulink', 'Analytical');


%%