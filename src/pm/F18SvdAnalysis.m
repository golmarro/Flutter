%Data for the F-18 HARV lateral directional dynamics at M = 0.38. H=5000 ft
%and alpha = 5 deg
%From The Control Handbook Chapter 38 (Eigenstructure Assignment)
%Model is augmented with first order dynamics actuators and yaw rate
%washout filter

%states:
%   delta_a - aileron deflection
%   delta_s - stabilator deflection
%   delta_r - rudder deflection
%   beta    - sideslip angle
%   p       - roll rate
%   r       - yaw rate
%   fi      - bank angle
%   x8      - washut filter state
%
%inputs:
%   delta_ac - aileron command
%   delta_sc - stabilator command
%   delta_sr - rudder command
%
%outputs
%   r_wo    - washed out yaw rate
%   p       - state roll rate
%   beta    - state sideslip angle
%   fi      - state bank angle

%All quantities are referenced in body axis frame units degrees or deg/sec

A = [-30      0        0         0          0        0       0       0
      0      -30       0         0          0        0       0       0
      0       0       -30        0          0        0       0       0
     -0.007  -0.014    0.0412   -0.1727     0.0873  -0.9946  0.0760  0
     15.3225  12.0601  2.2022   -11.0723   -2.1912   0.7096  0       0
     -0.3264  0.2041  -1.3524    2.1137    -0.0086  -0.1399  0       0
      0       0        0         0          1        0.0875  0       0
      0       0        0         0          0        0.5     0      -0.5];

B = [30     0       0
     0      30      0
     0      0       30
     0      0       0
     0      0       0
     0      0       0
     0      0       0
     0      0       0];

C = [0  0   0   0   0   1   0  -1
     0  0   0   0   1   0   0   0
     0  0   0   1   0   0   0   0
     0  0   0   0   0   0   1   0];

D = zeros(4,3);

f18 = ss(A,B,C,D);
set(f18,'InputName',{'aileron' ; 'stabilator' ; 'rudder'},'OutputName', ...
    {'wo yaw rate' ; 'roll rate' ; 'sideslip angle' ; 'bank angle'});

%% SvdAnalysis

SvdAnalysis(f18([2,3],:));
subplot(2,1,1);
legend('\sigma_{max} (G_{cl}(j \omega))','p [deg/s]','\beta [deg]')
title('Wzmocnienie uk³adu w najgorszym przypadku')
xlabel('rad/s');
subplot(2,1,2);
legend('\delta_{a} [deg]','\delta_s [deg]','\delta_r [deg]');
title('Kierunek wejœciowy maksymalnego wzmocnienia');
xlabel('rad/s');