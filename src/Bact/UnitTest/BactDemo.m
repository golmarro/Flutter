clear
figure
params = BactParams('metric');
wing = WingFlutter(params);
wing.SpeedRootLocus(100:20:700);
title('Linia pierwiastkowa od predkosci, dla roznych wysokosci');
xlabel('real')
ylabel('imag')