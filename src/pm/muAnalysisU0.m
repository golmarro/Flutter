
%% Model z parametryczna niepewnoscia z wykorzystaniem niepewnych atomow
%  U0 + fuel level

paramsUnc = WingUnc;
wing = WingFlutter(paramsUnc);
wing.U0 = paramsUnc.getU0_unc;
sys_uf = wing.getLinearModel;
sys_uf_simple = simplify(sys_uf, 'full');

%% Prostszy przypadek - tylko fuel level

paramsUnc = WingUnc;
wing = WingFlutter(paramsUnc);
%wing.U0 = paramsUnc.getU0_unc;
sys_f = wing.getLinearModel;
sys_f_simple = simplify(sys_f, 'full');

%%
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

%% Prostszy przypadek - tylko U0 Real

wing = WingFlutter();
paramsUnc = WingUnc;
wing.U0 = paramsUnc.getU0_unc;
sys_ureal = wing.getLinearModel;
sys_ureal_sim = simplify(sys_ureal,'full');     % Po simplify mamy zamiast 104 tylko 4  wystapienia U0!

%% Probki z muss idealnie sie pokrywaja
wing = WingFlutter();
for u = 40:20:120
    wing.U0 = u;
    tmp_n = wing.getLinearModel;
    tmp_u = usubs(sys_ureal_sim, 'U0',u);
    sigma(tmp_n(1,1),'k');
    hold on;
    sigma(tmp_u(1,1),'r--');
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


%% Tylko U0 ale complex

wing = WingFlutter();
paramsUnc = WingUnc;
U_avg = (paramsUnc.U0_min + paramsUnc.U0_max)/2;
rad = paramsUnc.U0_max - U_avg;
wing.U0 = ucomplex('U0',80,'Radius',rad);
sys_ucom = wing.getLinearModel;
sys_ucom_sim = simplify(sys_ucom,'full');

%%
for u = 40:20:120
    tmp_u = usubs(sys, 'U0',u*j);
    sigma(tmp_u(1,1),'y:');
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

wing = WingFlutter();
paramsUnc = WingUnc;
U_avg = (paramsUnc.U0_min + paramsUnc.U0_max)/2;
rad = paramsUnc.U0_max - U_avg;
wing.U0 = paramsUnc.U0_unc + ucomplex('U0_c',80*0.01,'Radius',80*0.01);
sys_umix = wing.getLinearModel;
sys_umix_simple = simplify(sys_umix, 'full');
%%
for u = 40:20:120
    tmp_u = usubs(sys_umix, 'U0',u, 'U_c', 80*0.01*j);
    sigma(tmp_u(1,1),'y:');
end

%% Wiemy ze niestabilnosc wystapi dla czestotliwosci flutteru ~ 44 rad/s
w = [40 : 1 :50];
sys_simple = simplify(sys,'full');
sys_fr = ufrd(sys_simple, w);
%%
opt = robopt('Display','on');
[stabmarg,destabu,report,info] = robuststab(sys_fr, opt);

%% Wyliczmy mu dla roznych czestosci pokazmy na wykresie
[Mfull DeltaFull] = lftdata(sys_ureal);
[Msim DeltaSim] = lftdata(sys_ureal_sim);
[Mmix DeltaMix] = lftdata(sys_umix_simple);
[Mcom DeltaCom] = lftdata(sys_ucom_sim);

nsim = size(DeltaSim,1);
nfull = size(DeltaFull,1);
nmix = size(DeltaMix,1);
ncom = size(DeltaCom,1);

%%
w = logspace(1, 4, 40);
sys_sim_fr = frd(Msim, w);
bounds_sim = mussv(sys_sim_fr(1:nsim, 1:nsim),[-nsim 0]);
sys_com_fr = frd(Mcom, w);
bounds_com = mussv(sys_com_fr(1:ncom, 1:ncom),[ncom 0]);
% sys_full_fr = frd(Mfull, w);
% bounds_full = mussv(sys_full_fr(1:nfull, 1:nfull),[nfull 0]);
sys_mix_fr = frd(Mmix, w);
bounds_mix = mussv(sys_mix_fr(1:nmix, 1:nmix),[-nmix/2 0; nmix/2 0]);

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
