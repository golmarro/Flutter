%% Compare WingSim, WingFlutter and Linear model with step on delta (no
%% gravity)
close_system('WingSim', 0);
wing = wingFlutter;
wingParams = wing.params;
simRunner = SimRunner('simRunner', wing);
wing.inputSignal = WingFlutter.constSignal(1);
wing.simStyle = 'r--';

figure
wing.isGravity = 'off';
%wing.AeroForces = 'off';

simRunner.sim(4);
wing.sim(4);

sys = wing.getLinearModel;
[y t] = step(sys([1 2],1),4);
subplot(2,1,1)
plot(t,y(:,1),'g:');
subplot(2,1,2)
plot(t,180/pi*y(:,2),'g:');

legend('WingSim', 'WingFlutter', 'Linear');
subplot(2,1,1);
title('Compare models for step input on delta (no gravity)');


