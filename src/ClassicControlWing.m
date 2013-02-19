%% 
clear
wing = WingFlutter;
plane = PlaneParams(wing.params);
plane.fuelLevel = 0.5;
plane.payloadLevel = 0.5;
wing.atmosphere.h = 4000;

Uf = wing.getFlutterSpeed;
wing.U0 = 1.4*Uf;
sys = wing.getModelSS;


%% theta_dot -> delta
sys2 = sys(4,1);
% system has one pair of unstable poles - to achieve stability point (-1,0)
% has to be encircled counter-clockwise once
% From nyquist plot we can see that it can be achieved using positice
% feedback of gain = 1;
nyquist(-sys2);


%% Bode plot
w = [1:0.04:12]*2*pi;
H = freqresp(sys2, w);
H = squeeze(H);
subplot(2,1,1); hold on;
plot(w/(2*pi), abs(H));
subplot(2,1,2); hold on;
plot(w/(2*pi), 180/pi*angle(H));

