clear
m1 = Model;
m2 = Model;

%% Model  z niepewnoscia niestrukturalna
%  Model niepewnosci wzglednej (multiplikatywnej):
%  Gp(s) = (I + w(s)Delta(s))Gn(s)

% Mamy juz model z niepewnoscia parametryczna, teraz sprobujmy dla skonczonego zbioru
% modeli nalezacych do rodziny Gr znalezc odpowiednia funkcje wagowa
% (skalarna transmitancje w(s)) tak, aby Gr nalezalo do Gp

Wunc = m2.unstructUncert(4);
title('');
xlabel('\omega [rad/s]');
ylabel('\sigma(G(j \omega))');

% Funkcja ucover domysle stosuje niepewnosc multiplikatywna na wejsciu -
% dla ukladow jednowymiarowych nie ma znaczenia czy na wejsciu czy na
% wyjsciu

%% Regulator Hinf bez kryterium stabilizacji
fprintf('Regulator #1 nie jest odpornie stabilny:\n');
m1.HinfSynthesis();

%% Regulator Hinf z dodanym kryterium stabilizacji
fprintf('Regulator #1 wyglada na odpornie stabilny:\n');
m2.HinfSynthesisStab();

% Chociaz dla reg #2 wartosc gamma < 1 nie zostala osiagnieta, to i tak
% samo uwzglednienie niepewnosci pozwolilo osiagnac odporna stabilnosc.

%% Sprawdzmy warunek wystarczajacy dla obydwu regulatorow
figure
sigma(1/m2.Wunc, m1.Khinf * m1.Cl, m2.Khinf * m2.Cl);
legend('1 / w_o(s)','GK(1 + GK)^{-1} reg. #1','GK(1 + GK)^{-1} reg. #2');
title('');
xlabel('\omega [rad/s]');
ylabel('\sigma(G(j \omega))');
%title('Mimo ze wartosc \gamma > 1 dla reg #2, to uklad odpornie stabilny')

% Warunek wystarczajacy stabilnosci nie zostal spelniony w zadnym przypadku:
% |GK(1+GK)|inf << |1/w(s)|
% Mimo to uklad #2 jest stabilny odpornie

%% Sprawdzmy co nie pozwolilo kontynuowac optymalizacji
SvdAnalysis(m2.Clw);

%% Synteza regulatora Mu

% Funkcja dksyn potrzebuje na wejsciu modelu z niepewnymi atomami (uss).
% Mozemy ja zbudowac na dwa sposoby:
%  * Od poczatku z uzyciem niepewnych atomow (funkcja Model.Gn_uss)
%  * Do modelu z wyodrebniona niepewnoscia (Model.Gdelta) dodac tylko
%    macierz Delta (funkcja Model.Gn_delta)
fprintf('W pierwszym podejsciu niepewne atomy wystepuja 5 razy\n')
m1.Gn_uss
fprintf('Nie pomaga tutaj proba uproszczenia simplify(m1.Gn_uss,full))\n')
simplify(m1.Gn_uss,'full')
fprintf('W drugim podejsciu niepewne atomy wystepuja 3 razy (tyle powinno byc)\n')
m1.Gn_delta

%% Zasadniczo modele sa identyczne
figure; hold on;
sigma(m.getModelStack, 'k');

delta = m1.getDelta();
for i=1:size(delta,1)
    tmp_m = normalized2actual(m1.m_unc,delta(i,1));
    tmp_c = normalized2actual(m1.c_unc,delta(i,2));
    tmp_k = normalized2actual(m1.k_unc,delta(i,3));
    Gu1 = usubs(m1.Gn_uss, 'm', tmp_m, 'c', tmp_c, 'k', tmp_k);
    sigma(Gu1, 'b-.')
end
for i=1:size(delta,1)
    Gu2 = usubs(m1.Gn_delta, 'dm', delta(i,1), 'dc', delta(i,2), 'dk', delta(i,3));
    sigma(Gu2, 'r:')
end

%% Synteza regulatora dla dwoch modeli
fprintf('W pierwszym podejsciu zbieznosc algorytmu jest fatalna, nie da sie dobrego wyniku uzyskac\n')
[K, CL, bnd, dkinfo] = m1.muSynthesis(m1.Gn_uss);
% Iteration Summary                                          
% -----------------------------------------------------------
% Iteration #                 1         2         3         4
% Controller Order            4        54        66        66
% Total D-Scale Order         0        50        62        62
% Gamma Acheived          4.494     3.971    14.699    15.409
% Peak mu-Value           3.005     2.742     2.855     3.351


fprintf('W pierwszym podejsciu zbieznosc algorytmu jest fatalna, nie da sie dobrego wyniku uzyskac\n')
[K, CL, bnd, dkinfo] = m1.muSynthesis(m1.Gn_delta);
% Iteration Summary                                
% -------------------------------------------------
% Iteration #                 1         2         3
% Controller Order            4        24        30
% Total D-Scale Order         0        20        26
% Gamma Acheived          3.508     1.569     6.869
% Peak mu-Value           2.552     1.486     1.899

%% Dodajemy stopien urojenia do niepewnosci
[K, CL, bnd, dkinfo] = m1.muSynthesis(m1.addComplexityToModel(0.01));
% Iteration Summary                                
% -------------------------------------------------
% Iteration #                 1         2         3
% Controller Order            4        28        36
% Total D-Scale Order         0        24        32
% Gamma Acheived          3.525     1.576    96.559
% Peak mu-Value           2.562     1.492     1.932


%% Analiza strukturalnej wartosci osobliwej dla regulatorow (odporna
% stabilnosc)

% open-loop:
boundsOl = m1.plotSsv();
% closed-loop
boundsHinf = m1.plotSsv(m1.Khinf);
boundsHinfStab = m2.plotSsv(m2.Khinf);
boundsMu = m1.plotSsv(m1.Kmu);

semilogx(boundsOl(1,1),'k',boundsHinf(1,1),'b--',boundsHinfStab(1,1),'g-.',boundsMu(1,1),'r-');
hold on;
semilogx([0.1 10],[1 1],'k:');
xlabel('\omega [rad/s]')
ylabel('\mu(G(j\omega))')
legend('open-loop','K_{H\infty}','Khinf 2','K_\mu')

%% Analiza odpornych osiagow

m1 = Model;
this = m1;
m1.HinfSynthesis
%%
m1.checkNominalPerformance(m1.Khinf);

%%
figure; hold on;
title('Wp criteria');

delta = m1.getDelta();
for i=1:size(delta,1)
    Gpert = lft(diag(delta(i,:)),m1.Gdelta);
    %m1.checkNominalPerformance(m1.Khinf, Gpert, gcf)
    G = augw(Gpert, m1.Wp, m1.Wu);
    cl = lft(G,m1.Khinf);
    sigma(cl, 'b', {0.1 10})
end

%%
figure; hold on;
title('Wp criteria');
bodemag(1/this.Wp, 'k');

delta = m1.getDelta();
for i=1:size(delta,1)
    Gpert = lft(diag(delta(i,:)),m1.Gdelta);
    %m1.checkNominalPerformance(m1.Khinf, Gpert, gcf)
    G = augw(Gpert, 1, 1);
    cl = lft(G,m1.Khinf);
    bodemag(cl(1,1), 'b')
end



%%
figure; hold on;
title('Wu criteria');
bodemag(1/this.Wu, 'k');

delta = m1.getDelta();
for i=1:size(delta,1)
    Gpert = lft(diag(delta(i,:)),m1.Gdelta);
    %m1.checkNominalPerformance(m1.Khinf, Gpert, gcf)
    G = augw(Gpert, 1, 1);
    cl = lft(G,m1.Khinf);
    
    bodemag(cl(2,1), 'b')
end

%%
m2 = Model;
m2.unstructUncert;
m2.HinfSynthesisStab;

%%
figure; hold on;
title('Wp criteria');

delta = m1.getDelta();
for i=1:size(delta,1)
    Gpert = lft(diag(delta(i,:)),m1.Gdelta);
    %m1.checkNominalPerformance(m1.Khinf, Gpert, gcf)
    G = augw(Gpert, m1.Wp, m1.Wu);
    cl = lft(G,m1.Khinf);
    sigma(cl, 'b', {0.1 10})
end

%%
this = Model;
this.unstructUncert
this.HinfSynthesisStab
%%
Gd = this.Gdelta;

Wp = this.Wp;
Wu = this.Wu;

systemnames = 'Gd Wu Wp';
inputvar = '[dm; dc; dk; dist; control]';
outputvar= '[Gd(1); Gd(2); Gd(3); Wp; Wu; Gd(4) + dist]';
input_to_Gd = '[dm; dc; dk; control]';
input_to_Wp = '[Gd(4) + dist]';
input_to_Wu = '[control]';
cleanupsysic = 'yes';
sysoutname = 'Pw';
sysic

%%
Gunc = this.Gn_delta;

G = this.getGsynth(this.Gn_uss);
[M Delta] = lftdata(G);
cl = lft(M, K);
fr = frd(cl, w);

block = [-1 0; -1 0; -3 0; 1 2];
[bounds, muinfo] = mussv(fr, block);

