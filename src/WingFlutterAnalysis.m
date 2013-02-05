path(path, 'Bact');
path(path, 'WingFlutter');

lbf_N = 4.44822162;
ft_m = 0.3048;
in_m = 0.0254;
slug_kg = 14.5939029;

%% Porownanie modelu Bact i Wing
clear
figure

% ---- initial conditions
alpha0 = 4*pi/180;
alpha0 = 0;
Gravity = 'on';
U0 = 35;   % [m/s]

% WingFlutter
wingParams = WingParams();
wingParams.alpha0 = alpha0;
wing = WingFlutter(wingParams);
wing.isGravity = Gravity;
wing.U0 = U0;
wing.simStyle = 'r--';
wing.sim(4);

% wing.params.Xcg_p = 0;
% wing.simStyle = 'g--';
% wing.sim(4);

% Bact
bactParams = BactParams('metric');
bactParams.alpha0 = alpha0;
wing = WingFlutter(bactParams);
wing.isGravity = Gravity;
wing.U0 = U0;
wing.simStyle = 'k--';
wing.sim(4);

legend('Wing','Bact');
title('przebiegi czasowe (delta skok 5 stopni)');


%% Predkosc flutteru dla roznych wysokosci
clear
figure; hold on;
params = WingParams();
wing = WingFlutter(params);
plane = PlaneParams(params);
plane.fuelLevel = 0;
wing.lineFormat = 'ko';
disp(params);
wing.flutterSpeedVersusAlt();
plane.fuelLevel = 0.3;
wing.lineFormat = 'r.';
disp(params);
wing.flutterSpeedVersusAlt();
plane.fuelLevel = 1;
wing.lineFormat = 'go';
disp(params);
wing.flutterSpeedVersusAlt();
legend('fuel level = 0%', 'fuel level = 30%', 'fuel level = 100%');

% Predkosc minimalna (zakladamy alpha max = 12 deg)
clear;
params = WingParams();
plane = PlaneParams();
wing = WingFlutter(params);
wing.isGravity = 'on';
%figure; 
hold on;
h = 0:100:11000;
Ustall = zeros(size(h));

for loadLevel = [0 0.5 1]
    plane.fuelLevel = loadLevel;
    plane.payloadLevel = loadLevel;
    
    for i = 1:length(h)    % [m]
        wing.atmosphere.h = h(i);
        Ustall(i) = sqrt(2 * (plane.totalMass/2 * wing.g) ./(wing.atmosphere.rho*wing.params.S*wing.params.alphaMax*wing.params.CLalpha));
    end
    
    plot(Ustall*3.6, h);
end

xlabel('U [km/h]');
ylabel('Altitude [m]');
title('Stall speed and flutter speed versus altitude')
legend('Load level = 0', 'Load level = 50%', 'Load level = 100%');
grid

%% Linia pierwiastkowa wzgledem predkosci dla roznych wysokosci
clear
figure
params = WingParams();
% params.Xsp_p = 0.5;
wing = WingFlutter(params);
wing.flutterSpeedVersusAlt();
wing.atmosphere.h = 0;
wing.SpeedRootLocus(25:5:110,'r.');
wing.atmosphere.h = 5000;
wing.SpeedRootLocus(25:5:160,'g.');
wing.atmosphere.h = 11000;
wing.SpeedRootLocus(25:5:200,'b.');
legend('h = 0 m', 'h = 5000 m', 'h = 11000 m');
ylabel('Imag')
xlabel('Real')
title('Root Locus versus speed')

%% Linia pierwiastkowa wzgledem predkosci dla roznej masy paliwa
clear
figure
params = WingParams();
plane = PlaneParams(params);
wing = WingFlutter(params);

plane.fuelLevel = 0;
l1 = sprintf('Xcg = %f %%, Itheta: %f, Mass: %f', params.Xcg_p*100, params.Itheta, params.mass);
wing.SpeedRootLocus(25:5:120,'r.');

plane.fuelLevel = 0.5;
l2 = sprintf('Xcg = %f %%, Itheta: %f, Mass: %f', params.Xcg_p*100, params.Itheta, params.mass);
wing.SpeedRootLocus(25:5:130,'g.');

plane.fuelLevel = 1;
l3 = sprintf('Xcg = %f %%, Itheta: %f, Mass: %f', params.Xcg_p*100, params.Itheta, params.mass);
wing.SpeedRootLocus(25:5:140,'b.');

legend(l1,l2,l3);
ylabel('Imag')
xlabel('Real')
title('Root Locus versus speed')

%% Czestotliwosc flutteru
clear
params = WingParams();
wing = WingFlutter(params);
plane = PlaneParams(params);

plane.fuelLevel = 0;
Uf = wing.getFlutterSpeed;
wing.U0 = Uf;
sys = wing.getModelSS();
[wn z] = damp(sys);
omegaF = wn(find(z<=0.001, 1));
fprintf('Flutter frequency for U = %f km/h: %f rad/s, %f Hz\n', wing.U0*3.6, omegaF, omegaF/2/pi);

plane.fuelLevel = 0.3;
Uf = wing.getFlutterSpeed;
wing.U0 = Uf;
sys = wing.getModelSS();
[wn z] = damp(sys);
omegaF = wn(find(z<=0.001, 1));
fprintf('Flutter frequency for U = %f km/h: %f rad/s, %f Hz\n', wing.U0*3.6, omegaF, omegaF/2/pi);

plane.fuelLevel = 1;
Uf = wing.getFlutterSpeed;
wing.U0 = Uf;
sys = wing.getModelSS();
[wn z] = damp(sys);
omegaF = wn(find(z<=0.001, 1));
fprintf('Flutter frequency for U = %f km/h: %f rad/s, %f Hz\n', wing.U0*3.6, omegaF, omegaF/2/pi);
%% Znieksztalcenie plata pod wlasnym ciezarem
clear;
params = WingParams();
plane = PlaneParams(params);
wing = WingFlutter(params);
wing.isGravity = 'on';
wing.AeroForces = 'off';
plane.fuelLevel = 0;
state = wing.trim(0,0);
fprintf('Pod wlasnym ciezarem (f=0): h = %f [m], theta = %f [deg]\n', state(1), state(2)*180/pi);
plane.fuelLevel = 0.5;
state = wing.trim(0,0);
fprintf('Pod wlasnym ciezarem (f=0.5): h = %f [m], theta = %f [deg]\n', state(1), state(2)*180/pi);
plane.fuelLevel = 1;
state = wing.trim(0,0);
fprintf('Pod wlasnym ciezarem (f=1): h = %f [m], theta = %f [deg]\n', state(1), state(2)*180/pi);


%% Statyczne skrecenie i wygiecie plata dla g = 1
% Dwa podejscia:
% #1 - zadana sila nosna dla totalMass/2

clear;
params = WingParams();
plane = PlaneParams();
wing = WingFlutter(params);
wing.isGravity = 'on';

% Print alpha versus speed
figure; hold on;
styles = ['r','g','b'];
for h = 1000    % [m]
    wing.atmosphere.h = h;
    loadLevel = [0 0.5 1];
    for loadIndex = [1 2 3]
        plane.fuelLevel = loadLevel(loadIndex);
        plane.payloadLevel = loadLevel(loadIndex);
        
        Ustall = sqrt(2 * (plane.totalMass/2 * wing.g) ./(wing.atmosphere.rho*wing.params.S*wing.params.alphaMax*wing.params.CLalpha));
        fprintf('Ustall: %f [km/h]\n', Ustall*3.6);
        U0 = (Ustall*3.6:20:400)/3.6;  % [m/s]
        fprintf('------------------ Total Mass: %f kg\n', plane.totalMass);
        fprintf('V [km/h]  alpha0[deg]   h [m]    theta [deg]    g [-]\n');
        alpha0 = [];
        for i = 1:length(U0)
            wing.U0 = U0(i);
            
            fprintf('%f\t', wing.U0*3.6)
            a0 = wing.trimLift(-plane.totalMass/2 * wing.g);     % # 2
            state = wing.trim(a0, 0);
            fprintf('%f\t%f\t%f\t', a0*180/pi, state(1), state(2)*180/pi);
            fprintf('%f\n', wing.q*wing.params.S*(state(2)+a0)*wing.params.CLalpha / (plane.totalMass/2 * wing.g));
            alpha0(i) = a0;
        end
        plot(U0*3.6, alpha0*180/pi, styles(loadIndex));
    end
end
xlabel('V [km/h]');
ylabel('\alpha [deg]');
title('Kat natarcia wzgledem predkosci');
legend('load level = 0', 'load level = 0.5', 'load level = 1');
grid

%% Postacie drgan dla U << Uf i dla U ~= Uf
clear;
% --------------------------- Wing
wingParams = WingParams();
plane = PlaneParams(wingParams);
wing = WingFlutter(wingParams);
wing.isGravity = 'on';

% U << Uf
Ustall = sqrt(2 * (plane.totalMass/2 * wing.g) ./(wing.atmosphere.rho*wing.params.S*wing.params.alphaMax*wing.params.CLalpha));
Uf = wing.getFlutterSpeed;
wing.U0 = 1.2 * Ustall;

fprintf('------------------------Wing U = %f Uf\n', wing.U0 / Uf);
plane.fuelLevel = 0;
wing.showModes;
plane.fuelLevel = 0.5;
wing.showModes;
plane.fuelLevel = 1;
wing.showModes;

% U ~= Uf
wing.U0 = 0.9*Uf;
fprintf('------------------------Wing U = %f Uf\n', wing.U0 / Uf);
plane.fuelLevel = 0;
wing.showModes;
plane.fuelLevel = 0.5;
wing.showModes;
plane.fuelLevel = 1;
wing.showModes;

% --------------------------- BACT
clear
bactParams = BactParams('metric');
wing = WingFlutter(bactParams);
wing.isGravity = 'on';

% U << Uf
alphaMax = 12*pi/180;
Uf = wing.getFlutterSpeed;
wing.U0 = Uf * 0.4;

fprintf('------------------------ U = %f Uf\n', wing.U0 / Uf);
wing.showModes;

% U ~= Uf
wing.U0 = 0.9*Uf;
fprintf('------------------------ U = %f Uf\n', wing.U0 / Uf);
wing.showModes;


%% Czestosci drgan skretnych i gietnych
clear
params = WingParams();
plane = PlaneParams(params);
plane.fuelLevel = 0;
fprintf('Czestosc drgan skretnych: omega theta = %f [rad/s]\n', params.omegatheta);
fprintf('Czestosc drgan gietnych : omega h     = %f [rad/s]\n', params.omegah);
plane.fuelLevel = 0.5;
fprintf('Czestosc drgan skretnych: omega theta = %f [rad/s]\n', params.omegatheta);
fprintf('Czestosc drgan gietnych : omega h     = %f [rad/s]\n', params.omegah);
plane.fuelLevel = 1;
fprintf('Czestosc drgan skretnych: omega theta = %f [rad/s]\n', params.omegatheta);
fprintf('Czestosc drgan gietnych : omega h     = %f [rad/s]\n', params.omegah);

