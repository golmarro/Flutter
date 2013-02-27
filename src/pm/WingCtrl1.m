%% Predkosc flutteru dla roznych wysokosci
clear
figure; hold on;
params = WingParams();
wing = WingFlutter(params);
plane = PlaneParams(params);
plane.fuelLevel = 0;
wing.lineFormat = 'r';
disp(params);
wing.flutterSpeedVersusAlt();
plane.fuelLevel = 0.3;
wing.lineFormat = 'r:';
disp(params);
wing.flutterSpeedVersusAlt();
plane.fuelLevel = 1;
wing.lineFormat = 'r--';
disp(params);
wing.flutterSpeedVersusAlt();
%legend('fuel level = 0%', 'fuel level = 30%', 'fuel level = 100%');
legend('0%', '30%', '100%');

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
styles = {'b','b:','b--'};
loadLevels = [0 0.3 1];

for l = 1:length(loadLevels) 
    loadLevel = loadLevels(l);
    plane.fuelLevel = loadLevel;
    plane.payloadLevel = loadLevel;
    
    for i = 1:length(h)    % [m]
        wing.atmosphere.h = h(i);
        Ustall(i) = sqrt(2 * (plane.totalMass/2 * wing.g) ./(wing.atmosphere.rho*wing.params.S*wing.params.alphaMax*wing.params.CLalpha));
    end
    
    plot(Ustall*3.6, h, styles{l});
end

xlabel('U [km/h]');
ylabel('Altitude [m]');
title('Zale¿noœæ prêdkoœci Vmin i Vk wzglêdem wysokoœci i poziomu paliwa')
%legend('Load level = 0', 'Load level = 50%', 'Load level = 100%');
grid

%% Czestosci drgan skretnych i gietnych
clear
params = WingParams();
plane = PlaneParams(params);
fprintf('poziom paliwa [%%] & c_cg [%%] & m [kg] & Itheta [kg] & omega_h [rad/s] & omega_theta [rad/s]\n');

for fuelLevel = 0:0.25:1
    plane.fuelLevel = fuelLevel;
    fprintf('%.0f & %3.2f & %0.2f & %.2f & %.2f & %.2f \\\\ \n',plane.fuelLevel*100,params.Xcg_p * 100, plane.wingMass, plane.wingItheta, params.omegah, params.omegatheta);
end