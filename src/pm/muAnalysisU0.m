% Analiza strukturalnej wartosci osobliwej (dla modelu gdzie niepewnym
% parametrem jest tylko U0)
clear
%% U0 Real

wing = WingFlutter();
paramsUnc = WingUnc;
wing.U0 = paramsUnc.getU0_unc;
sys_ureal = wing.getLinearModel;
sys_ureal_simple = simplify(sys_ureal,'full');     % Po simplify mamy zamiast 104 tylko 4  wystapienia U0!

%% U0 complex

wing = WingFlutter();
paramsUnc = WingUnc;
U_avg = (paramsUnc.U0_min + paramsUnc.U0_max)/2;
rad = paramsUnc.U0_max - U_avg;
wing.U0 = ucomplex('U0',80,'Radius',rad);
sys_ucom = wing.getLinearModel;
sys_ucom_simple = simplify(sys_ucom,'full');

%% U0  = real + 0,01*complex

wing = WingFlutter();
paramsUnc = WingUnc;
U_avg = (paramsUnc.U0_min + paramsUnc.U0_max)/2;
rad = paramsUnc.U0_max - U_avg;
wing.U0 = paramsUnc.U0_unc + ucomplex('U0_c',80*0.01,'Radius',80*0.01);
sys_umix = wing.getLinearModel;
sys_umix_simple = simplify(sys_umix, 'full');

%% Probki z muss idealnie sie pokrywaja
wing = WingFlutter();
for u = 80
    wing.U0 = u;
    tmp_n = wing.getLinearModel;
    tmp_ureal = usubs(sys_ureal_simple, 'U0',u);
    tmp_mix = usample(usubs(sys_umix_simple, 'U0',u), 5);
    sigma(tmp_n(1,1),'k',{20 100});
    hold on;
    %sigma(tmp_ureal(1,1),'g--',{20 100});
    sigma(tmp_mix(1,1),'b--',{20 100});
end
title('');
xlabel('\omega (rad/s)')
ylabel('\sigma_{max}(G)');
legend('\delta_U = 0','\delta_U = 0 + j\delta_{UC}' );

%% Wyliczmy mu (SSV) dla roznych czestosci, pokazmy na wykresie problemy z
%  obliczaniem SSV gdy Delta sklada sie z liczb rzeczywistych
[Mfull DeltaFull] = lftdata(sys_ureal);
[Msim DeltaSim] = lftdata(sys_ureal_simple);
[Mmix DeltaMix] = lftdata(sys_umix_simple);
[Mcom DeltaCom] = lftdata(sys_ucom_simple);

nsim = size(DeltaSim,1);
nfull = size(DeltaFull,1);
nmix = size(DeltaMix,1);
ncom = size(DeltaCom,1);

%%
w = logspace(1, 2, 400);
%w = [20 30 35 40:0.4:48 50 60 80];
sys_sim_fr = frd(Msim, w);
bounds_sim = mussv(sys_sim_fr(1:nsim, 1:nsim),[-nsim 0]);
sys_com_fr = frd(Mcom, w);
bounds_com = mussv(sys_com_fr(1:ncom, 1:ncom),[ncom 0]);
% sys_full_fr = frd(Mfull, w);
% bounds_full = mussv(sys_full_fr(1:nfull, 1:nfull),[nfull 0]);
%sys_mix_fr = frd(Mmix, w);
%bounds_mix = mussv(sys_mix_fr(1:nmix, 1:nmix),[-nmix/2 0; nmix/2 0]);


%% Recznie wprowadzimy maly stopien urojenia :)
Fin = [eye(nsim), 0.01*eye(nsim)];
Fout = Fin';
Msim2 = Fout*Msim(1:nsim,1:nsim)*Fin;
sys_mix2_fr = frd(Msim2, w);
bounds_mix2 = mussv(sys_mix2_fr,[-nsim 0; nsim 0]);


%% Wiemy ze niestabilnosc wystapi dla czestotliwosci flutteru ~ 44 rad/s
w = [40 : 1 :50];
sys_simple = simplify(sys,'full');
sys_fr = ufrd(sys_simple, w);

%% Pokazemy dolne oszacowanie ktore niedobrze wychodzi
h1 = loglog(bounds_sim(1,1),'b--',bounds_sim(1,2),'b');
hold on;
h2 = loglog(bounds_mix2(1,1),'r--',bounds_mix2(1,2),'r');
xlabel('\omega [rad/s]')
ylabel('\mu_\Delta(G(j\omega))')
legend([h1(1) h2(1)],'Niepewnoœæ rzeczywista','Niepewnosc urojona' );


%% Pokazemy gorne oszacowanie mu oraz pik odpowiadajacy pulsacji flatteru
figure
semilogx(bounds_sim(1,1),'k');
hold on;
semilogx([10 100],[2.34 2.34],'k--');
semilogx([10 100],[1 1],'k:');
xlabel('\omega [rad/s]')
ylabel('gorne oszacowanie \mu_\Delta(G(j\omega))')



