% Rozdzial - Model Samolotu
% - parametry modelu (za³¹cznik)
% - model sztwny - postacie ruchu
% - model polsztywny

%% Parametry
% minimalna, maksymalna masa
clear
plane = PlaneParams;
plane.fuelLevel = 0;
plane.payloadLevel = 0;
disp(plane);
plane.fuelLevel = 1;
plane.payloadLevel = 1;
disp(plane);

%% Maksymalne statyczne znieksztalcenia plata