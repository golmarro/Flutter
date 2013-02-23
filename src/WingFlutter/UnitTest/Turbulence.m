
%% Test turbulence models (state-space and transfer function)
clear
params = WingParams;
Ttf = params.getTurbulenceTF;
Tss = params.getTurbulence;

%% Test turbulence embedded in Full Linear Model
wing = WingFlutter;
sys = wing.getLinearModel;

figure;
step(Ttf, 'b', Tss, 'k:',sys([6,7],2), 'r:')
legend('TF','SS','Full model');

%% Examine influence of turbulence on h and theta 
% - compare it with step response to delta
figure
subplot(2,1,1)
step(sys([1 2], 2));
subplot(2,1,2)
step(sys([1 2], 1));

%% Compare WingSim and WingFlutter with step on turbulence input
clear
wing = WingFlutter;
%close_system('WingSim', 0);
wingParams = wing.params;
simRunner = SimRunner('simRunner', wing);
wing.inputSignal = WingFlutter.constSignal(0);
wing.turbulenceInputSignal = WingFlutter.constSignal(1);
%wing.simStyle = 'r--';
wing.isGravity = 'off';
%wing.AeroForces = 'off';

figure
simRunner.sim(4);
% wing.sim(4);

sys = wing.getLinearModel;
[y t] = step(sys([1 2],2),4);
subplot(2,1,1)
plot(t,y(:,1),'g:');
subplot(2,1,2)
plot(t,180/pi*y(:,2),'g:');

%legend('WingSim', 'WingFlutter', 'Linear');
legend('WingSim', 'Linear');
subplot(2,1,1);
title('Compare models for step input on turbulence (no gravity)');


