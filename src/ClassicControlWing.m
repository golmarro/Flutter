%% 
clear
wing = WingFlutter;
plane = PlaneParams(wing.params);
plane.fuelLevel = 0.5;
plane.payloadLevel = 0.5;
wing.atmosphere.h = 4000;

Uf = wing.getFlutterSpeed;
wing.U0 = Uf;
sys = wing.getLinearModel;

%% Znak sprzezenia zwrotnego
% Z podstawowej analizy szybko mozna sie zorientowac, ze potrzebne jest
% dodatnie sprzezenie zwrotne (wynika to z przyjetej konwencji znakow)
% Chociaz jest to niestandardowa konwencja, autor postanowil zastosowac ja
% dla latwiejszej nalizy porownawczej

%% Wybor sygnalu
% Standardowo uzywanymi przyrzadami pomiarowymi w praktycznych
% eksperymentach tlumienia flutteru sa przyspieszeniomierze. Sygnal z dwoch
% przyspierzeniomierzy umieszczonych na krawedziach skrzydla, po
% odpowiednim przeskalowaniu odpowiadaja wartoscia \( \ddot{h(t)} \) i \(
% \ddot{\theta(t)} \).
fprintf('Pierwiastki ukladu otwartego\n');
damp(sys)

%% washout
% Z analizy wykonanej w [Vivek] wynika, ze najlepszym sygnalem do
% regulacj SISO jest theta_dot, dlatego musimy calkowac sygnal.
% Zeby nie wzmocnic stalego bledu przyspieszeniomierza, musimy zastosowac
% washout s/(s+a)
% razem z calkowaniem otrzymujemy zwykly lag compensator

%          a              ________         ________                 
%   F = -------          /                         \
%        s + a          /                           \
%                         washout         1/s * washout = lag

% Czestotliwosc flutteru wynosi 6,67 Hz, czestotliwosc filtra washout
% dobrano tak, vby nie wplywal na sygnal w poblizu czestotliwosci flutteru
% - o rzad wielkosci mniejszy
% w_w = 2*pi*6.67/10
w_w = 4.19;             % rad/s
Fw = tf(w_w,[1 w_w]);

%% Analiza linii pierwiastkowej
% Z wykresu odczytujemy wartosc wzmocnienia odpowiadajaca najwieszemu
% wspolczynnikowi tlumienia.
figure
rlocus(Fw*(-sys(9,1)));
hold on;
%rlocus(-sys(4,1));
axis([-50,10,-60,60])
K = 0.2;
title('Linia pierwiastkowa F_w(s) G_{\theta}(s)');
%% Analiza metoda Nyquista
% Poniewaz uklad otwarty posiada dwa niestabilne pierwiastki, wykres
% Nyquista odpowiedzi amplitudowo-fazowej ukladu powinien otoczyc punk
% (-1,0) jednokrotnie w kierunku przeciwnym do ruchu wskazowek zegara.

figure
nyquist(K*Fw*(-sys(9,1)));
title('Wykres amplitudowo-fazowy uk³adu otwartego');
% zblizenie
figure
nyquist(K*Fw*(-sys(9,1)));
title('Wykres amplitudowo-fazowy uk³adu otwartego');
axis([-2,2,-2,2])
%hold on
%washout2 = tf(10,[1 10]);
%nyquist(0.16*washout2*(-sys(9,1)));
%legend('washout1','washout2')
%% Analiza wykresu Bodego
figure
bode(K*Fw*(-sys(9,1)));
grid

%% Bode plot
% w = [1:0.04:12]*2*pi;
% H = freqresp(sys2, w);
% H = squeeze(H);
% subplot(2,1,1); hold on;
% plot(w/(2*pi), abs(H));
% subplot(2,1,2); hold on;
% plot(w/(2*pi), 180/pi*angle(H));

%% Analysis
K = -K*Fw;
K.InputNames = sys.OutputNames(9);
K.OutputNames = sys.InputNames(1);

analysis = LinearAnalysis(K);
analysis.resetDefaultConditions();
analysis.turbulenceAnalysis(1);
U1 = analysis.U0;

analysis.lineColor = 'r';
analysis.resetDefaultConditions();
Uf = analysis.wing.getFlutterSpeed();
analysis.U0 = Uf;
analysis.turbulenceAnalysis(1);
U2 = analysis.U0;

l1 = sprintf('U = %.1f m/s (%.0f%% U_f)', U1, U1/Uf * 100);
l2 = sprintf('U = %.1f m/s (%.0f%% U_f)', U2, U2/Uf * 100);
legend(l1,l2);
