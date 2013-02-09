
% ----------------------------------------------- Stiff plane model

%% Verify trimming
clear
close_system('PlaneStiffSim',0)
simRunner = PlaneRunner('simRunner', 'PlaneStiffSim');
simRunner.actuatorModel = 'none';

%
[t1 y1] = simRunner.sim(1);
simRunner.trim();
[t2 y2] = simRunner.sim(1);


%
figure
simRunner.lineStyle = 'r';
simRunner.plotEuler(t1, y1);
simRunner.lineStyle = 'b--';
simRunner.plotEuler(t2, y2);
legend('Before trim', 'after trim');

figure
simRunner.lineStyle = 'r';
simRunner.plotSpeed(t1, y1);
simRunner.lineStyle = 'b--';
simRunner.plotSpeed(t2, y2);
legend('Before trim', 'after trim');

%% Verify controls
clear
close_system('PlaneStiffSim',0)
simRunner = PlaneRunner('simRunner', 'PlaneStiffSim');
simRunner.actuatorModel = 'none';

%
simRunner.trim();
[t1 y1] = simRunner.sim(1);
simRunner.inputSignalType = 'pitch_up';
[t2 y2] = simRunner.sim(1);


%
figure
simRunner.lineStyle = 'r';
simRunner.plotEuler(t1, y1);
simRunner.lineStyle = 'b--';
simRunner.plotEuler(t2, y2);
legend('Constant input', 'Pitch up');

figure
simRunner.lineStyle = 'r';
simRunner.plotSpeed(t1, y1);
simRunner.lineStyle = 'b--';
simRunner.plotSpeed(t2, y2);
legend('Constant input', 'Pitch up');

%% Compare linear vs full model
clear
close_system('PlaneStiffSim',0)
simRunner = PlaneRunner('simRunner', 'PlaneStiffSim');
simRunner.actuatorModel = 'none';
simRunner.trim();
sys = simRunner.linearize();

%
simRunner.inputSignalType = 'pitch_up';

[t_full y_full] = simRunner.sim(1);
[y_lin t_lin] = lsim(sys, simRunner.u_plane, simRunner.t);

%
figure
simRunner.lineStyle = 'k';
simRunner.plotEuler(t_full, y_full);
simRunner.lineStyle = 'b--';
simRunner.plotEuler(t_lin, y_lin);
legend('Full', 'Linear');

figure
simRunner.lineStyle = 'k';
simRunner.plotSpeed(t_full, y_full);
simRunner.lineStyle = 'b--';
simRunner.plotSpeed(t_lin, y_lin);
legend('Full', 'Linear');
