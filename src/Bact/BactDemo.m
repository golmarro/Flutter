
%% 
bact = Bact(440, 125);
bact.SpeedRootLocus([100:20:700], 0.00237);
bact.SpeedRootLocus([100:20:900], 0.00088, 'g.');
bact.SpeedRootLocus([100:20:1100], 0.00046, 'c.');
title('Linia pierwiastkowa od predkosci, dla roznych wysokosci');
xlabel('real')
ylabel('imag')
legend('3000 ft', '30000 ft','45000 ft');