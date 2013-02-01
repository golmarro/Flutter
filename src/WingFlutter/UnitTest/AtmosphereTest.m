%% Test Atmosphere class
% Plot parameters variation
clear
Atmosphere.plot();
% Display parameters
fprintf('------------------------------------------------------- \n')
atm = Atmosphere(5000);
disp(atm);
fprintf('\n')
% Compare with standard
fprintf('------------------------------------------------------- \n')
fprintf('Compare with 1976 standard atmosphere for 5000 m altitude\n');
fprintf('                        1970 std.      this model\n');
fprintf('pressure        [Pa]    54022          %f\n',atm.p);
fprintf('density         [kg/m3] 0.736          %f\n',atm.rho);
fprintf('temp.           [K]     255.7          %f\n',atm.T);
fprintf('speed of sound  [m/s]   320.5          %f\n',atm.c);
fprintf('\n')
% Test setting other conditions than altitude
fprintf('------------------------------------------------------- \n')
atm = Atmosphere(5000);
fprintf('alt [m]\t\t\tpress [Pa]\t\t\trho [kg/m3]\t\ttemp [K]\t\tc [m/s]\n');
fprintf('%f\t\t%f\t\t%f\t\t%f\t\t%f\n', atm.h, atm.p, atm.rho, atm.T, atm.c);
p = atm.p; rho = atm.rho; T = atm.T; c = atm.c;
atm = Atmosphere();
atm.p = p;
fprintf('%f\t\t%f\t\t%f\t\t%f\t\t%f\n', atm.h, atm.p, atm.rho, atm.T, atm.c);
% atm = Atmosphere();
% atm.rho = rho;
% fprintf('%f\t\t%f\t\t%f\t\t%f\t\t%f\n', atm.h, atm.p, atm.rho, atm.T, atm.c);
atm = Atmosphere();
atm.T = T;
fprintf('%f\t\t%f\t\t%f\t\t%f\t\t%f\n', atm.h, atm.p, atm.rho, atm.T, atm.c);
atm = Atmosphere();
atm.c = c;
fprintf('%f\t\t%f\t\t%f\t\t%f\t\t%f\n', atm.h, atm.p, atm.rho, atm.T, atm.c);
fprintf('\n')