%% Wycinamy z kompletnego modelu tylko mdoel struktury plata
clear
wing = WingFlutter;
%wing.U0 = 0;
wing.isGravity = 'off';
wing.AeroForces = 'off';
wing.params.zetah = 0;
wing.params.zetatheta = 0;

wingSys = wing.getLinearModel;

sys = wingSys(1:4,1);
A = sys.a(1:4,1:4);
%B = sys.b(1:4,:);
%B = [0;0;1/wing.params.m;0];
B = [0;0;1/wing.params.m; -wing.params.shtheta/wing.params.m/wing.params.Itheta];
%C = sys.c(:,1:4);
C = eye(4);
sys = ss(A,B,C,sys.d);
sys.InputName = {'non'};
sys.StateName = {'h','theta','h_dot','theta_dot'};
sys.OutputName = {'h','theta','h_dot','theta_dot'};
sys

%% Tworzymy rodzine modeli strukturalnych skrzydla dla l = 0 : 0.1 : 1
clear Garray
wing = WingFlutter;
%wing.U0 = 0;
wing.isGravity = 'off';
wing.AeroForces = 'off';

for l = [0: 0.1: 1]
    wing.params.fuelLevel = l;
    wingSys = wing.getLinearModel;
    sys = wingSys(1:4,1);
    A = sys.a(1:4,1:4);
    %B = sys.b(1:4,:);
    %B = [0;0;1;0];
    %B = [0;0;1/wing.params.m;0];
    B = [0;0;1/wing.params.m; -wing.params.shtheta/wing.params.m/wing.params.Itheta];
    %C = sys.c(:,1:4);
    C = eye(4);
    sys = ss(A,B,C,sys.d);
    sys.InputName = {'non'};
    sys.StateName = {'h','theta','h_dot','theta_dot'};
    sys.OutputName = {'h','theta','h_dot','theta_dot'};

    if ~exist('Garray','var')
        Garray = stack(1,sys);
    else
        Garray = stack(1,sys,Garray);
    end
end

sigma(Garray,'k');

%% Wyprowadzamy model jeszcze raz - dla sprawdzenia
m = wing.params.m;
I = wing.params.Itheta;
s = wing.params.shtheta;

M = [m s; s I];
%Minv = [I -s; -s m] * (1/(m*I - s*s));
%Minv2 = inv(M);

Kh = wing.params.Kh;
Ktheta = wing.params.Ktheta;
K = [Kh 0; 0 Ktheta];

%
A = [zeros(2)    eye(2);
     -inv(M)*K   zeros(2)];
%B = [0;0;1;0];
%B = [0;0;1/wing.params.m;0];
B = [0;0;1/wing.params.m; -wing.params.shtheta/wing.params.m/wing.params.Itheta];
C = eye(4);
D = zeros(4,1);

sys2 = ss(A,B,C,D);

%% Model z niepewnoscia bezwzgledna
clear
wing = WingFlutter;
%wing.U0 = 0;
wing.isGravity = 'off';
wing.AeroForces = 'off';

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
     -inv(M)*K   -inv(M)*Ds];
%B = [0;0;1;0];
%B = [0;0;1/m;0];
B = [0;0;1/m; -s/m/I];
C = eye(4);
D = zeros(4,1);

uncSys = ss(A,B,C,D);

%% Model z parametryczna niepewnoscia z wykorzystaniem niepewnych atomow
clear
paramsUnc = WingUnc;
wing = WingFlutter(paramsUnc);
wing.U0 = paramsUnc.getU0_unc;
sys = wing.getLinearModel;

%% Prostszy przypadek - tylko fuel level
clear
paramsUnc = WingUnc;
wing = WingFlutter(paramsUnc);
%wing.U0 = paramsUnc.getU0_unc;
sys = wing.getLinearModel;
opt = robopt('Display','on');
[stabmarg,destabu,report, info] = robuststab(sys, opt);

% Niestety z poziomem paliwa jest problem, bo wiadomo ze ujemna wartosc nie
% ma sensu, a tutaj wlasnie ujemna wartosc zostala wskazana jako
% destabilizujaca

% Uncertain System is robustly stable to modeled uncertainty.                                  
%  -- It can tolerate up to 181% of the modeled uncertainty.                                   
%  -- A destabilizing combination of 208% of the modeled uncertainty exists,                   
%     causing an instability at 1.75e-016 rad/s.                                               
%  -- Sensitivity with respect to uncertain element ...                                        
%    'FuelLevel' is 106%.  Increasing 'FuelLevel' by 25% leads to a 27% decrease in the margin.
% 
% stabmarg1=
%                 UpperBound: 2.0812
%                 LowerBound: 1.8087
%     DestabilizingFrequency: 1.7483e-016
% destabu1=
%     FuelLevel: -0.5406

%% Prostszy przypadek - tylko U0
clear
paramsUnc = WingUnc;
wing = WingFlutter();
wing.U0 = paramsUnc.getU0_unc;
sys = wing.getLinearModel;
%%
opt = robopt('Display','on');
[stabmarg,destabu,report,info] = robuststab(sys, opt);

% Ten raport jest niestety bzdurny, bo zwykly test:
% step(sys(1,1),1)
% pokazuje typowo niestabilne przebiegi czasowe

% Uncertain System is robustly stable to modeled uncertainty.                    
%  -- It can tolerate up to 189% of the modeled uncertainty.                     
%  --   No modeled uncertainty exists to cause an instability at 9.34e-019 rad/s.
%  -- Sensitivity with respect to uncertain element ...                          
%    'U0' is 141%.  Increasing 'U0' by 25% leads to a 35% decrease in the margin.
% 
% disp(stabmarg)
%                 UpperBound: Inf
%                 LowerBound: 1.8892
%     DestabilizingFrequency: 9.3422e-019
% 
% disp(destabu)
%     U0: 4.4102e+052

%% mussv tests

M = [1 j;1+2j -2];
M = rand(2) + j*rand(2);
%  | d1  0  |
%  | 0   d2 |
% d1, d2 in C
blk = [-1 0; -1 0];
[bounds, muinfo] = mussv(M,blk);
bounds

%% Przyklad z "Control Handbook" str 674 (692)
M1 = [0.1 	-0.154 		0 		0;
0		-0.3		2.86	-26;
0.1		0.077		-0.4	5;
0		-0.004		0.006	0.2;
0.024	0			-0.066	0];
M2 = [0.07		0.162		-0.56	42;
-0.273		-0.28		0.546	72.8;
0.175		-0.108		0.21	3.5;
0.002		-0.002		0.011	0.42;
0.028		0.027		0.042	0.7];
M = M1 + j*M2;
% Przy okazji wplyw struktury delta na wynik:
blk = [1 0; 1 0; 2 3];
[bounds, muinfo] = mussv(M,blk);
bounds
blk = [1 0; 1 0; 1 0; 1 2];
[bounds, muinfo] = mussv(M,blk);
bounds
% Zgadza sie z tym co w ksiazce

[VDelta,VSigma,VLmi] = mussvextract(muinfo);
fprintf('Wielkosc macierzy zaburzen Delta odpowiada odwrotnosci dolnego ograniczenia _mu = 1/delta_max\n');
disp([norm(VDelta) 1/bounds(2)])
fprintf('Macierz VDelta rzeczywiscie destabilizuje uklad\n');
fprintf('det(I-M Delta) = ');
disp(det(eye(size(M*VDelta))-M*VDelta))