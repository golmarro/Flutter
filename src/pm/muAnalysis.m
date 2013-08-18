
%% Model z parametryczna niepewnoscia z wykorzystaniem niepewnych atomow
paramsUnc = WingUnc;
wing = WingFlutter(paramsUnc);
wing.U0 = paramsUnc.getU0_unc;
sys_uf = wing.getLinearModel;
sys_uf_simple = simplify(sys_uf,'full');

%% Analiza SSV dla pelnego modelu
[Muf DeltaUf] = lftdata(sys_uf_simple);
nuf = size(DeltaUf,1);
w = logspace(1, 2, 100);

Fin = [eye(nuf), 0.01*eye(nuf)];
Fout = Fin';
Muf2 = Fout*Muf(1:nuf,1:nuf)*Fin;
sys_uf_fr = frd(Muf2, w);
[bounds_uf2 mu_info] = mussv(sys_uf_fr,[-nuf 0; nuf 0]);



%% Prostszy przypadek - tylko fuel level
clear
paramsUnc = WingUnc;
wing = WingFlutter(paramsUnc);
%wing.U0 = paramsUnc.getU0_unc;
sys = wing.getLinearModel;
sys_simple = simplify(sys,'full');
%%
opt = robopt('Display','on');
[stabmarg,destabu,report, info] = robuststab(sys, opt);

% sys
%-----
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

% sys_simple
%------------
% Uncertain System is possibly NOT robustly stable to modeled uncertainty.                   
%  -- It can tolerate up to 35% of the modeled uncertainty.                                  
%  -- A destabilizing combination of 208% of the modeled uncertainty exists,                 
%     causing an instability at 1.45e-007 rad/s.                                             
%  -- Sensitivity with respect to uncertain element ...                                      
%    'FuelLevel' is 34%.  Increasing 'FuelLevel' by 25% leads to a 9% decrease in the margin.
%    'U0' is 96%.  Increasing 'U0' by 25% leads to a 24% decrease in the margin.             
% 
% destabu = 
% 
%     FuelLevel: -0.5406
%            U0: 33.9497

%% Prostszy przypadek - tylko U0 Real
clear
wing = WingFlutter();
paramsUnc = WingUnc;
wing.U0 = paramsUnc.getU0_unc;
sys = wing.getLinearModel;
sys_sim = simplify(sys,'full');     % Po simplify mamy zamiast 104 tylko 4  wystapienia U0!
%% Probki z muss idealnie sie pokrywaja
wing = WingFlutter();
for u = 40:20:120
    wing.U0 = u;
    sys_n = wing.getLinearModel;
    sys_u = usubs(sys_sim, 'U0',u);
    sigma(sys_n(1,1),'k');
    hold on;
    sigma(sys_u(1,1),'r--');
end
%%
opt = robopt('Display','on');
[stabmarg,destabu,report,info] = robuststab(sys_sim, opt);

% Raport dla systemu bez simplify:
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

% Raport dla systemu po simplify: - duzo szybciej sie liczy, ale nadal
% bzdurny
% Uncertain System is robustly stable to modeled uncertainty.                    
%  -- It can tolerate up to 270% of the modeled uncertainty.                     
%  -- A destabilizing combination of 270% of the modeled uncertainty exists,     
%     causing an instability at 1e-008 rad/s.                                    
%  -- Sensitivity with respect to uncertain element ...                          
%    'U0' is 100%.  Increasing 'U0' by 25% leads to a 25% decrease in the margin.

% destabu = 
% 
%     U0: 187.9202

%% Wyliczmy mu dla roznych czestosci pokazmy na wykresie
[Mfull DeltaFull] = lftdata(sys);
[Msim DeltaSim] = lftdata(sys_sim);

nsim = size(DeltaSim,1);
nfull = size(DeltaFull,1);

w = logspace(1, 4, 40);
sys_sim_fr = frd(Msim, w);
bounds_sim = mussv(sys_sim_fr(1:nsim, 1:nsim),[nsim 0]);
% sys_full_fr = frd(Mfull, w);
% bounds_full = mussv(sys_full_fr(1:nfull, 1:nfull),[nfull 0]);

%% Tylko U0 ale complex
clear
wing = WingFlutter();
paramsUnc = WingUnc;
U_avg = (paramsUnc.U0_min + paramsUnc.U0_max)/2;
rad = paramsUnc.U0_max - U_avg;
wing.U0 = ucomplex('U0',80,'Radius',rad);
sys = wing.getLinearModel;

%%
for u = 40:20:120
    sys_u = usubs(sys, 'U0',u*j);
    sigma(sys_u(1,1),'y:');
end

%%
opt = robopt('Display','on');
[stabmarg,destabu,report,info] = robuststab(sys, opt);

% Uncertain System is NOT robustly stable to modeled uncertainty.                
%  -- It can tolerate up to 33.2% of the modeled uncertainty.                    
%  -- A destabilizing combination of 33.2% of the modeled uncertainty exists,    
%     causing an instability at 45.5 rad/s.                                      
%  -- Sensitivity with respect to uncertain element ...                          
%    'U0' is 301%.  Increasing 'U0' by 25% leads to a 75% decrease in the margin.

% destabu = 
% 
%     U0: 87.9960 +10.5896i

% Wynik: algorytm znalazl niestabilnosc dla U0 = 88 + 10i.
% Ale uklad jest stabilny dla U0 = 88, jest dopiero niestabilny dla U0 = 97

% ZESPOLONA warosc parametru wprowadza duzy konserwatyzm

%% Tylko U0 ale real + 0,01*complex   ------- Nic z tego za ciezkie
% obliczeniowo
clear
wing = WingFlutter();
paramsUnc = WingUnc;
U_avg = (paramsUnc.U0_min + paramsUnc.U0_max)/2;
rad = paramsUnc.U0_max - U_avg;
wing.U0 = paramsUnc.U0_unc + ucomplex('U0_c',80*0.01,'Radius',80*0.01);
sys = wing.getLinearModel;

%%
for u = 40:20:120
    sys_u = usubs(sys, 'U0',u*j);
    sigma(sys_u(1,1),'y:');
end

%% Wiemy ze niestabilnosc wystapi dla czestotliwosci flutteru ~ 44 rad/s
w = [40 : 1 :50];
sys_simple = simplify(sys,'full');
sys_fr = ufrd(sys_simple, w);
%%
opt = robopt('Display','on');
[stabmarg,destabu,report,info] = robuststab(sys_fr, opt);

%% Recznie wprowadzimy maly stopien urojenia :)
clear
wing = WingFlutter();
paramsUnc = WingUnc;
wing.U0 = paramsUnc.getU0_unc;
sys = wing.getLinearModel;

[M, Delta] = lftdata(sys);



%% "Odrealnianie" niepewnych oparametrow
blkrsR = [-1 1;-1 1;-1 1];
rob_stab = sel(clp_g,[1:3],[1:3]);
pdim = ynum(rob_stab);
fixl = [eye(pdim); 0.1*eye(pdim)]; % 1% Complex
fixr = fixl';
blkrs = [blkrsR; abs(blkrsR)];
clp_mix = mmult(fixl,rob_stab,fixr);
[rbnds,rowd,sens,rowp,rowg] = mu(clp_mix,blkrs);
