%% Check units conversion metric - english
clear

lbf_N = 4.44822162;
ft_m = 0.3048;
in_m = 0.0254;
slug_kg = 14.5939029;

% ------ initial conditions
alpha0 = 4*pi/180;
U0 = 200;   % [ft/s]

% ------ metric
params = BactParams('metric');
params.alpha0 = alpha0;
wing = WingFlutter(params);
wing.isGravity = 'off';
wing.U0 = U0 * ft_m;
% q is taken from Atmosphere class

[t y] = wing.sim(4);

subplot(2,1,1)
plot(t, y(:,1)/ft_m, 'r');
subplot(2,1,2)
plot(t, y(:,2)*180/pi, 'r');

% ------ english (old Bact class)
bactParams = BactParams('CLinverse');
bactParams.alpha0 = alpha0;
bact = Bact2(bactParams);
bact.isGravity = 'off';
bact.U0 = U0;
bact.q = wing.q * 1/(lbf_N / ft_m^2);
bact.simStyle = 'k--';
bact.sim(4);
