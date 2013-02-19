% Classic control system synthesis based on 
% Mukhpadhyay V.: Transonic Flutter Suppression Control Law Design Using
% Classical and Optimal Techniques with Wind-Tunnel Results, 1999

% Reduced 4th order state space equations in air 225 psf

F = [ -1.6073  21.001 0       0;
      -21.001 -1.6073 0       0;
        0      0      0.7515 25.167;
        0      0    -25.167   0.7515];

G  = [-3.8259; 12.713; -2.2202; 4.1351];
Gw = [0.0597 ; 0.272; -0.1107; -0.1745];

Hd = [-0.0517 -0.0132 -0.0668  0.0063;
      -0.0542 -0.009  -0.0482 -0.0016;
       0       0       0       0     ;
       0       0       0       0     ;
       0       0       0       0     ;
      10.378   0.6369  7.9924  0.0671;
       0.3897 -0.9373 -2.7625  0.9909];

Edu = [0.0440; 0.0421; 1; 50; 0;       0.5758;  0.0358];
Edw = [0.0002; 0.0004; 0;  0; 0.0968; -0.0004; -0.0003];

% xdot = F x + Gw w + G u
%   yd = Hd x + Edw w + Edu u        - design output
%   ys = H x  + Esw w + Esu u        - sensor output ??
%
% outputs: [zte zle dte ddte gust lift moment]
% inputs:  [delta w]

sys = ss(F, [G Gw], Hd, [Edu Edw]);
sys.InputNames = ['delta [deg]';
                  'w [ft/s]   '];
sys.OutputNames =['zte [g]     ';
                  'zle [g]     ';
                  'dte [deg]   ';
                  'ddte [deg/s]';
                  'gust [ft/s] ';
                  'lift [lbf]  ';
                  'moment      '];

%% Plot response to step signal on delta trailing edge input
figure
title('Open-Loop response to 1 deg. TE input');
subplot(2,2,1)
step(sys(1,1),2)
ylabel('zte [g]')

subplot(2,2,2)
step(sys(2,1),2)
ylabel('zle [g]')

subplot(2,2,3)
step(sys(6,1),2)
ylabel('lift [lb]')

subplot(2,2,4)
step(sys(7,1),2)
ylabel('moment [lb-ft]')

%% Bode diagrams

figure
systems = {sys(1,1), sys(2,1), sys(1,1)-sys(2,1)};
styles = ['r','g','b'];
for i = 1 : length(systems)
    w = [1:0.04:8]*2*pi;
    H = freqresp(systems{i}, w);
    H = squeeze(H);
    subplot(2,1,1); hold on;
    plot(w/(2*pi), abs(H), styles(i));
    subplot(2,1,2); hold on;
    plot(w/(2*pi), 180/pi*angle(H), styles(i));
end

subplot(2,1,1);
title('Bode diagrams of zte, zle and (zte - zle) due to \delta_{te} (q = 225 psf, M = 0.5)');
subplot(2,1,2);
xlabel('freq Hz');

%% Chosen feedback signal (zte - zle)
tmp = [1 -1 0 0 0 0 0];
sys2 = tmp*sys;
nyquist(sys2, sys2*25);
axis equal

CL1 = feedback(sys2, 25, 1, 1);
damp(CL1)
bode(CL1)

%% Final Pitch and Pitch-Rate Feedback Control Law  - Controller 3

K3 = ss([-10 0; 5 -5], [10 -4; 0 0], [50 1], 0);
% K.InputNames = ['delta [deg]';

CL3 = feedback(sys,K,1,[1 2]);


%% Plot response to step signal on delta trailing edge input - Controller 3
figure
title('Closed-Loop response to 1 deg. TE input - Controller 3');
subplot(2,2,1)
step(CL3(3,1),2)
ylabel('dte [deg]')

subplot(2,2,2)
step(CL3(4,1),2)
ylabel('dte rate [deg/s]')

subplot(2,2,3)
step(CL3(6,1),2)
ylabel('lift [lb]')

subplot(2,2,4)
step(CL3(7,1),2)
ylabel('moment [lb-ft]')
