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