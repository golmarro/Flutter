
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
close_system('PlaneSim',0)
simRunner = PlaneRunner('simRunner', 'PlaneStiffSim');
simRunner.actuatorModel = 'none';
wing = WingFlutter;
Vk = wing.getFlutterSpeed;
simRunner.wingFlutter.U0 = Vk*1;
simRunner.trim();
sys = simRunner.linearize();
sys.StateName = {'theta','psi','-phi','x1','x2','x3','d_theta','d_psi','-d_phi','v1','v2','v3'};
%%
%Postaci ruchu


%%
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

%% -------------------------------------------- Flexible model
clear
close_system('PlaneStiffSim',0)
simRunner = PlaneRunner('simRunner', 'PlaneSim');
% simRunner.actuatorModel = 'none';
wing = WingFlutter;
Vk = wing.getFlutterSpeed;
simRunner.wingFlutter.U0 = Vk*0.75;
simRunner.trim();
sys = simRunner.linearize();
sys = sys(1:14,:);
%sys.StateName = {'theta','psi','-phi','x1','x2','x3','d_theta','d_psi','-d_phi','v1','v2','v3'};

%% Root locus
clear
close_system('PlaneStiffSim',0)
close_system('PlaneSim',0)
wing = WingFlutter;
Vk = wing.getFlutterSpeed;
P = [0.6:0.05:1.2];
simRunner = PlaneRunner('simRunner', 'PlaneSim');
simRunner.actuatorModel = 'none';
sys = {};
for p = P
    simRunner.wingFlutter.U0 = Vk*p;
    simRunner.trim();
    sys{end+1} = simRunner.linearize();
end
%%
wing = WingFlutter;
wSys = {};
for p = P
    wing.U0 = Vk*p;
    wSys{end+1} = wing.getLinearModel;
end
%% plot locus
styles = {'r','k','k','k','k','k','k','k','k','k','k','k','b','k'};

for i = 1:length(sys)
    pzmap(sys{i},styles{i})
    %pzmap(sys{end})
    hold on;
    
    [v l] = eig(wSys{i}.a);
    l = diag(l);
    plot(real(l),imag(l),[styles{i} 'o']);
end
    
