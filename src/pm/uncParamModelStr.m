% Proba recznego wyprowadzenia modelu z wyodrebniona niepewnoscia
% parametryczna ( delta - G )
% Niestety nadal roznica jest spora, gdzies sa bledy


%% Tworzymy rodzine modeli strukturalnych skrzydla na podstawie wyjsciowego
%  modelu WingFlutter
clear;
wing = WingFlutter;
wing.params.fuelLevel = 0.5;
wing.isGravity = 'off';
wing.AeroForces = 'off';

for l = [0: 0.5: 1]
    wing.params.fuelLevel = l;
    wingSys = wing.getLinearModel;
    sys = wingSys(1:4,1);
    A = sys.a(1:4,1:4);
    %B = sys.b(1:4,:);
    %B = [0;0;1;0];
    %B = [0;0;1/wing.params.m;0];
    M = wing.Ms();
    B = [zeros(2); -inv(M)];
    %B = [0;0;1/wing.params.m; -wing.params.shtheta/wing.params.m/wing.params.Itheta];
    %C = sys.c(:,1:4);
    C = [eye(2) zeros(2)];
    D = zeros(2);
    sys = ss(A,B,C,D);
    sys.InputName = {'Q_h','Q_theta'};
    sys.StateName = {'h','theta','h_dot','theta_dot'};
    sys.OutputName = {'h','theta'};

    if ~exist('Garray','var')
        Garray = stack(1,sys);
    else
        Garray = stack(1,sys,Garray);
    end
end

sigma(Garray(1,1),'k--');
hold on;

%% Wyprowadzamy model nominalny jeszcze raz - dla sprawdzenia recznie
wing.params.fuelLevel = 0.5;
m = wing.params.m;
I = wing.params.Itheta;
s = wing.params.shtheta;

M = [m s; s I];
Kh = wing.params.Kh;
Ktheta = wing.params.Ktheta;
K = [Kh 0; 0 Ktheta];
D = wing.Ds;

A = [zeros(2)    eye(2);
     -inv(M)*K   -inv(M)*D];
B = [zeros(2); -inv(M)];
C = [eye(2) zeros(2)];
D = zeros(2);

sysNom1 = ss(A,B,C,D);
sigma(sysNom1(1,1),'b--');

%% Model z niepewnoscia bezwzgledna z wykorzystaniem niepewnych atomow
%clear
wing.params.fuelLevel = 0;
qmin = [wing.params.m; wing.params.Itheta; wing.params.shtheta];
wing.params.fuelLevel = 1;
qmax = [wing.params.m; wing.params.Itheta; wing.params.shtheta];

% Macierz tlumienia strukturalnego
Ds = wing.Ds;

fl = ureal('FuelLevel',0.5,'Range',[0 1]);
m = qmin(1) + fl*(qmax(1) - qmin(1));
I = qmin(2) + fl*(qmax(2) - qmin(2));
s = qmin(3) + fl*(qmax(3) - qmin(3));
M = [m s; s I];
Kh = wing.params.Kh;
Ktheta = wing.params.Ktheta;
K = [Kh 0; 0 Ktheta];

A = [zeros(2)    eye(2);
    %-inv(M)*K    zeros(2)];
     -inv(M)*K   -inv(M)*Ds];
B = [zeros(2); -inv(M)];
C = eye(2,4);
D = zeros(2,2);

uncSys = ss(A,B,C,D);
sigma(usubs(uncSys(1,1),'FuelLevel',0), 'r--');
hold on;
sigma(usubs(uncSys(1,1),'FuelLevel',0.5), 'r--');
sigma(usubs(uncSys(1,1),'FuelLevel',1), 'r--');
%% Ustrukturyzowana Niepewnosc Parametryczna (Na przykladzie
% mass-spring-damper z [Robust with Matlab]) czyli reczne wyprowadzenie
% modelu
%clear;
wing = WingFlutter;
wing.params.fuelLevel = 0;
qmin = [wing.params.m; wing.params.Itheta; wing.params.shtheta];
wing.params.fuelLevel = 1;
qmax = [wing.params.m; wing.params.Itheta; wing.params.shtheta];

m = (qmax(1) + qmin(1))/2;
I = (qmax(2) + qmin(2))/2;
s = (qmax(3) + qmin(3))/2;

pm = (qmax(1) - qmin(1))/(qmax(1) + qmin(1));
pI = (qmax(2) - qmin(2))/(qmax(2) + qmin(2)); 
ps = (qmax(3) - qmin(3))/(qmax(3) + qmin(3));

wing.params.fuelLevel = 0.5;
Ds = wing.Ds;

%det = 1/(m*I-s*s);
M = [m s; s I];
Kh = wing.params.Kh;
Ktheta = wing.params.Ktheta;
K = [Kh 0; 0 Ktheta];


% A nie zmienia sie
A = [zeros(2)    eye(2);
     -inv(M)*K   -inv(M)*Ds];
    %-inv(M)*K   zeros(2)];

 % Tutaj recznie wyznaczylem macierz B, ale okazalo sie ze mozna to zapisac
 % macierzowo
%dM = det(M);

%B_h_dd =     [I    -s   -m*I*pm    pI*s*I   -ps*I    ps*s] * det(M);
%B_theta_dd = [-s   m     s*m*pm   -pI*m*I    ps*s   -ps*m] * det(M);

% Zapis macierzowy:
%    um    uI                    us1   us2     F    M    
B = [0    0                      0     0       0     0;
     0    0                      0     0       0     0;
    -inv(M)* [pm*m 0;0 pI*I]    -inv(M)*ps     inv(M)];

 C = [
     -inv(M)*K - [Kh/m 0; 0 Ktheta/I]     zeros(2); % ym, yI
     -inv(M)*K*s                          zeros(2); % ys1 ys2
     1 0 0 0;              % h
     0 1 0 0];              % theta
      
  
%    um    uI   us1   us2     F    M
D =  [
     s/m*inv(M)*[pm*m 0;0 pI*I]-[pm 0;0 pI]  s/m*inv(M)*ps - [ps/m 0; 0 ps/I]  -s/m*inv(M)+[1/m 0;0 1/I] ;
     -s*inv(M)* [pm*m 0;0 pI*I]                      -s*inv(M)*ps               s*inv(M)     ;
      0 0 0 0 0 0;                      % h
      0 0 0 0 0 0];                      % theta

sys_u = ss(A,B,C,D);
sys_u.InputName = {'um','uI','us1','us2','F','M'};
sys_u.OutputName = {'ym','yI','ys1','ys2','h','theta'};
sys_u.StateName = {'h','theta','h_dot','theta_dot'};

% Wygenerujemy skrajne przypadki
for d = [-0.5 0 0.5]
    delta = diag([d d d d]);
    sys = lft(delta, sys_u);
    sigma(sys(1,1),'b--');
    hold on;
end