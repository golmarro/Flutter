%% 
clear
wing = WingFlutter;
% Model without actuator
sys1 = wing.getModelSS;
% Model with actuator
sys2 = wing.getLinearModel;

% Get flutter speed
Uf = wing.getFlutterSpeed;
% Set half of flutter speed
wing.U0 = 0.5*Uf;

figure
subplot(2,1,1)
step(sys2(5,1));
title('Delta response to delta control');

subplot(2,1,2);
step(sys1(1,1),'k',sys2(1,1),'r:');
title('Effect of actuator');