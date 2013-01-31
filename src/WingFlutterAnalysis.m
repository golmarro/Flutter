path(path, 'c:\ROBUST\Flutter\Bact\');

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
U0 = 50;   % [m/s]

% WingFlutter
wingParams = WingParams('CLinverse');
wingParams.alpha0 = alpha0;
wing = WingFlutter(wingParams);
wing.isGravity = Gravity;
wing.U0 = U0;
wing.simStyle = 'r--';
wing.sim(4);

wing.params.Xcg_p = 0;
wing.simStyle = 'g--';
wing.sim(4);

% Bact
params = BactParams('CLinverse metric');
params.alpha0 = alpha0;
wing = WingFlutter(params);
% wing.isGravity = 'off';
wing.U0 = U0;
wing.simStyle = 'k--';
[t y] = wing.sim(4);

legend('Wing', 'Wing Xcg=0','Bact');
Title('przebiegi czasowe (delta skok 5 stopni)');


%% Predkosc flutteru dla roznych wysokosci
clear
figure; hold on;
params = WingParams('CLinverse');
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
params = WingParams('CLinverse');
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
        Ustall(i) = sqrt(2 * (plane.totalMass/2 * wing.g) ./(-wing.atmosphere.rho*wing.params.S*wing.params.alphaMax*wing.params.CLalpha));
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
params = WingParams('CLinverse');
% params.Xsp_p = 0.5;
wing = WingFlutter(params);
wing.flutterSpeedVersusAlt();
wing.atmosphere.h = 0;
wing.SpeedRootLocus(25:5:130,'r.');
wing.atmosphere.h = 5000;
wing.SpeedRootLocus(25:5:180,'g.');
wing.atmosphere.h = 11000;
wing.SpeedRootLocus(25:5:240,'b.');
legend('h = 0 m', 'h = 5000 m', 'h = 11000 m');
ylabel('Imag')
xlabel('Real')
title('Root Locus versus speed')

%% Linia pierwiastkowa wzgledem predkosci dla roznej masy paliwa
clear
figure
plane = PlaneParams;
params = WingParams('CLinverse');
wing = WingFlutter(params);

plane.fuelLevel = 0;
params.Xcg_p = plane.wingXcg_p;
params.Itheta = plane.wingItheta;
params.mass = plane.wingMass;
l1 = sprintf('Xcg = %f %%, Itheta: %f, Mass: %f', params.Xcg_p*100, params.Itheta, params.mass);
wing.SpeedRootLocus(25:5:180,'r.');

plane.fuelLevel = 0.5;
params.Xcg_p = plane.wingXcg_p;
params.Itheta = plane.wingItheta;
params.mass = plane.wingMass;
l2 = sprintf('Xcg = %f %%, Itheta: %f, Mass: %f', params.Xcg_p*100, params.Itheta, params.mass);
wing.SpeedRootLocus(25:5:180,'g.');

plane.fuelLevel = 1;
params.Xcg_p = plane.wingXcg_p;
params.Itheta = plane.wingItheta;
params.mass = plane.wingMass;
l3 = sprintf('Xcg = %f %%, Itheta: %f, Mass: %f', params.Xcg_p*100, params.Itheta, params.mass);
wing.SpeedRootLocus(25:5:150,'b.');

legend(l1,l2,l3);
ylabel('Imag')
xlabel('Real')
title('Root Locus versus speed')

%% Czestotliwosc flutteru
clear
params = WingParams('CLinverse');
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
params = WingParams('CLinverse');
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
% #2 - zadane sila na mocowaniu K*x dla massPerWing

clear;
params = WingParams('CLinverse');
plane = PlaneParams();
wing = WingFlutter(params);
wing.isGravity = 'on';

% Print alpha versus speed
figure; hold on;
% Theta  = [];
for h = 1000    % [m]
    wing.atmosphere.h = h;

    for loadLevel = [0 0.5 1]
        plane.fuelLevel = loadLevel;
        plane.payloadLevel = loadLevel;
        
        Ustall = sqrt(2 * (plane.totalMass/2 * wing.g) ./(-wing.atmosphere.rho*wing.params.S*wing.params.alphaMax*wing.params.CLalpha));
        fprintf('Ustall: %f [km/h]\n', Ustall*3.6);
        U0 = (Ustall*3.6:20:400)/3.6;  % [m/s]
        fprintf('------------------ Total Mass: %f kg\n', plane.totalMass);
        fprintf('V [km/h]  alpha0[deg]   h [m]    theta [deg]    g [-]    alpha0[deg]     h [m]     theta [deg]  g [-]\n');
        alpha0 = [];
        for i = 1:length(U0)
            wing.U0 = U0(i);
            
            fprintf('%f\t', wing.U0*3.6)
            
%             a0 = wing.trimForce(-plane.massPerWing * wing.g);    % # 1
%             state = wing.trim(a0, 0);
%             fprintf('%f\t%f\t%f\t', a0*180/pi, state(1), state(2)*180/pi);
%             fprintf('%f\t', state(1) * wing.params.Kh / (plane.massPerWing * wing.g));
            
            a0 = wing.trimLift(-plane.totalMass/2 * wing.g);     % # 2
            state = wing.trim(a0, 0);
            fprintf('%f\t%f\t%f\t', a0*180/pi, state(1), state(2)*180/pi);
            fprintf('%f\n', wing.q*wing.params.S*(state(2)+a0)*wing.params.CLalpha / (plane.totalMass/2 * wing.g));
            alpha0(i) = a0;
        end
        plot(U0, alpha0*180/pi);
    end
end
%wing.inputSignal = WingFlutter.constSignal();
%wing.params.alpha0 = a0;
%wing.sim(10,[state; 0; 0]);

% alpha0_2 = wing.trimLift(-mass * wing.g);
% fprintf('Kat natarcia dla m = %f kg, V = %f km/h: alpha0 = %f [deg]\n', mass, wing.U0*3.6, alpha0_1*180/pi)
% fprintf('h = %f m, theta = %f deg\n', state(1), state(2)*180/pi);

%% Postacie drgan dla U << Uf i dla U ~= Uf
clear;
% --------------------------- Wing
wingParams = WingParams('CLinverse');
plane = PlaneParams(wingParams);
wing = WingFlutter(wingParams);
wing.isGravity = 'on';

% U << Uf
Ustall = sqrt(2 * (plane.totalMass/2 * wing.g) ./(-wing.atmosphere.rho*wing.params.S*wing.params.alphaMax*wing.params.CLalpha));
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
bactParams = BactParams('CLinverse metric');
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
params = WingParams('clinverse');
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

