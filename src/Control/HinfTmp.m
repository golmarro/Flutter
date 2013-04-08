%% Get full model for flutter speed
clear
h_max = 0.1;
theta_max = 3*pi/180;
delta_max = 45*pi/180;


wing = WingFlutter;
wing.U0 = wing.getFlutterSpeed();
G = wing.getLinearModel();

%% Analysis model
P = G([1,2,8,9],[1 1]);
fprintf('Inputs:\n');
disp(P.InputName)
fprintf('Outputs:\n')
disp(P.OutputName)

P.InputName = ['delta_p';'delta_c'];
%% Remove turbulence model
% fprintf('Inputs:\n')
% disp(G.InputName(1))
% fprintf('Outputs:\n')
% disp(G.OutputName(1))
% s2 = G([1:5, 8:9],[1]);
% A = s2.A(1:6,1:6);
% B = s2.B(1:6,:);
% C = s2.C(:,1:6);
% D = s2.D;
% P = ss(A,B,C,D);

%% Leave only  [ h theta h_ddot theta_ddot ]

% P.InputName = [G.InputName(1)];
% P.StateName = s2.StateName(1:6);
% P.OutputName = s2.OutputName();
% P = P([1 2 6 7],:); % h theta h_ddot theta_ddot

%% Add weights
Wu = delta_max*0.3;
Wp = [h_max 0;
      0   theta_max];

systemnames = 'P Wp Wu';
inputvar = '[delta_p;delta_c]';
outputvar = '[Wp;Wu;P(3:4)]';
input_to_P = '[delta_p;delta_c]';
input_to_Wp = '[P(1:2)]';
input_to_Wu = '[delta_c]';
%outputnames = '[zp{2};zu;h_dd;theta_dd]';
Pw = sysic;
  
%Pw = augw(P,Wp, Wu, []);

%% Synth
[Khinf, CL] = hinfsyn(Pw, 2, 1, 'display', 'on');

cl = feedback(P, Khinf,1,3:4, 1);

%%
systemnames = '[P Khinf]';
inputvar = '[delta_p]';
outputvar = '[P(1:2);Khinf]';
input_to_P = '[delta_p;Khinf]';
input_to_Khinf = '[P(3:4)]';
cl2 = sysic;
