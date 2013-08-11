path(path, 'WingFlutter');
path(path, 'Simulink');

%%
clear
close_system('WingSim', 0);
wingParams = WingParams();
wing = WingFlutter(wingParams);
simRunner = SimRunner('simRunner', wing);
wing.simStyle = 'r--';

%% Porownanie w prozni - tylko drgania strukturalne i grawitacja
figure
wing.isGravity = 'on';
wing.AeroForces = 'off';

simRunner.sim(2);
wing.sim(2);
legend('WingSim', 'model analityczny');
ylabel('\theta [deg]');
subplot(2,1,1);
ylabel('h [m]');
%title('Porownanie w prozni - tylko drgania strukturalne i grawitacja')

%% Bez grawitacji, w prozni, stan poczatkowy
figure
wing.isGravity = 'off';
wing.AeroForces = 'off';

state = [0 3*pi/180 0 0];
simRunner.sim(4, state);
wing.sim(4, state);
legend('Simulink', 'Analytical');
title('Bez grawitacji, w prozni, stan poczatkowy')
% Tutaj widac duze roznice - najprawdopodobniej chodzi o shtheta Mozemy to
% jednak zlekcewazyc, bo sily strukturalne maja mniejsze znaczenie

%% Skok jednostkowy na lotce
figure
wing.isGravity = 'on';
wing.AeroForces = 'on';

simRunner.sim(2);
wing.sim(2);
legend('WingSim', 'model analityczny');
ylabel('\theta [deg]');
subplot(2,1,1);
ylabel('h [m]');
%title('Skok jednostkowy na lotce')
% Roznice moga wynikac z samego sygnalu wejsciowego. W modelu simulink czas
% probkowania = 0.01. W modelu analitycznym moze byc duzo mniejszy

%% Bez grawitacji, Alpha 0
figure
wing.isGravity = 'off';
wing.AeroForces = 'on';
wing.setInputSignal('const');
wingParams.alpha0 = 3*pi/180;

simRunner.sim(4);
wing.sim(4);
legend('Simulink', 'Analytical');
title('Bez grawitacji, Alpha 0');
