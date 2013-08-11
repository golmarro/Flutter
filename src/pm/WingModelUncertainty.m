%% Niepewnosc modelu skrzydla
% clear;
% wing = WingFlutter;
% Vf = wing.getFlutterSpeed;
%[W1 W2 Gunc] = wing.addUncert(5, 1, 0.5, 5500, Vf*[0.8 1 1.4]);
%[W1 W2 Gunc] = wing.multUncert(5, 2, 0.5, 5500, Vf*[0.8 1 1.4]);

%% Rodzina modeli i model nominalny
% [RodzinaModeliGr.eps]
clear
figure
[Garray Gnom] = modelFamily(2);
legend('G_n(s)','G_i(s)');

%% Niepewnosc wzgledna na wyjsciu
clear
figure
[Garray Gnom] = modelFamily(0);
close(gcf)
figure
[Wunc,Info] = ucover(Garray,Gnom,3,[],'OutputMult');
[svGnom w] = sigma(Gnom);
svWo = sigma(Info.W1, w);
loglog(w, svGnom(1,:) + svWo .* svGnom(1,:), 'g:', 'LineWidth',1.5);
hold on;

[Wunc,Info] = ucover(Garray,Gnom,7,[],'OutputMult');
[svGnom w] = sigma(Gnom);
svWo = sigma(Info.W1, w);
loglog(w, svGnom(1,:) + svWo .* svGnom(1,:), 'm:', 'LineWidth',1.5);

[Wunc,Info] = ucover(Garray,Gnom,5,[],'OutputMult');
[svGnom w] = sigma(Gnom);
svWo = sigma(Info.W1, w);
loglog(w, svGnom(1,:) + svWo .* svGnom(1,:), 'b', 'LineWidth',2);

[Garray Gnom] = modelFamily(2);

legend('(I + W_o^3(s) \Delta(s))G_n(s)','(I + W_o^7(s) \Delta(s))G_n(s)',...
       '(I + W_o^5(s) \Delta(s))G_n(s)','G_n(s)','G_i(s)');

%% Niepewnosc wzgledna na wejsciu
clear
figure
[Garray Gnom] = modelFamily(0);
close(gcf)
figure
[Wunc,Info] = ucover(Garray,Gnom,[],3,'InputMult');
[svGnom w] = sigma(Gnom);
svWi = sigma(Info.W1,w);
loglog(w, svGnom(1,:) + svWi .* svGnom(1,:), 'g:', 'LineWidth',1.5);
hold on;

[Wunc,Info] = ucover(Garray,Gnom,[],7,'InputMult');
[svGnom w] = sigma(Gnom);
svWi = sigma(Info.W1,w);
loglog(w, svGnom(1,:) + svWi .* svGnom(1,:), 'm:', 'LineWidth',1.5);

[Wunc,Info] = ucover(Garray,Gnom,[],5,'InputMult');
[svGnom w] = sigma(Gnom);
svWi = sigma(Info.W1,w);
loglog(w, svGnom(1,:) + svWi .* svGnom(1,:), 'b', 'LineWidth',2);

[Garray Gnom] = modelFamily(2);
legend('G_n(s)(I + W_i^3(s) \Delta(s))', 'G_n(s)(I + W_i^7(s) \Delta(s))',...
       'G_n(s)(I + W_i^5(s) \Delta(s))','G_n(s)','G_i(s)');

%% Niepewnosc bezwzgledna
clear
figure
[Garray Gnom] = modelFamily(0);
close(gcf)
figure
[Wunc,Info] = ucover(Garray,Gnom,3,[],'Additive');
[svGnom w] = sigma(Gnom);
svWa = sigma(Info.W2,w);
loglog(w, svGnom(1,:) + svWa, 'g:', 'LineWidth',1.5);
hold on;

[Wunc,Info] = ucover(Garray,Gnom,7,[],'Additive');
[svGnom w] = sigma(Gnom);
svWa = sigma(Info.W2,w);
loglog(w, svGnom(1,:) + svWa, 'm:', 'LineWidth',1.5);

[Wunc,Info] = ucover(Garray,Gnom,5,[],'Additive');
[svGnom w] = sigma(Gnom);
svWa = sigma(Info.W2,w);
loglog(w, svGnom(1,:) + svWa, 'b', 'LineWidth',2);

[Garray Gnom] = modelFamily(2);
legend('G_n(s) + W_a^3(s) \Delta(s)', 'G_n(s) + W_a^7(s) \Delta(s)',...
       'G_n(s) + W_a^5(s) \Delta(s)','G_n(s)','G_i(s)');

%% Porownanie modeli niepewnosci   
clear
figure
[Garray Gnom] = modelFamily(0);
close(gcf)
figure;

[sv w] = sigma(Gnom);
loglog(w, sv(1,:), 'r', 'LineWidth',2);
hold on;

[Wunc,Info] = ucover(Garray,Gnom,5,[],'OutputMult');
[svGnom w] = sigma(Gnom);
svWo = sigma(Info.W1, w);
Wo = Info.W1;
loglog(w, svGnom(1,:) + svWo .* svGnom(1,:), 'b', 'LineWidth',1.5);
hold on;

[Wunc,Info] = ucover(Garray,Gnom,[],5,'InputMult');
[svGnom w] = sigma(Gnom);
svWi = sigma(Info.W1,w);
Wi = Info.W1;
loglog(w, svGnom(1,:) + svWi .* svGnom(1,:), 'm', 'LineWidth',1.5);

[Wunc,Info] = ucover(Garray,Gnom,7,[],'Additive');
[svGnom w] = sigma(Gnom);
svWa = sigma(Info.W2,w);
Wa = Info.W2;
loglog(w, svGnom(1,:) + svWa, 'g', 'LineWidth',1.5);

ylabel('\sigma_{max}(G)')
xlabel('\omega [rad/s]')
set(gca,'XLim',[4 10000])
set(gcf,'Units','centimeters','Position',[0, 0, 14, 8]);
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', [14 8]);

legend('G_n(s)','(I + W_o(s) \Delta(s))G_n(s)','G_n(s)(I + W_i(s) \Delta(s))',...
    'G_n(s) + W_a(s)');

%%

% Output Multiplicative:    USYS = (I + W1*ULTIDYN*W2)*PNOM
% ord W1 = 5
% ord W2 = []
[Wunc,Info] = ucover(Garray,Gnom,5,[],'OutputMult');

%%

% Get freq range
w = logspace(1,2.2,100);
%[svGnom w] = sigma(Gnom);

for i = 1 : length(Garray)
    %sigma(Garray(:,:,i),'k:');
    sv = sigma(Garray(:,:,i),w);
    semilogx(w, mag2db(sv), 'k:');
    hold on;
end
svGnom = sigma(Gnom, w);
svWo = sigma(Info.W1, w);

semilogx(w, mag2db(svGnom(1,:) + svWo .* svGnom(1,:)), 'b');
semilogx(w, mag2db(svGnom),'r')

sigma(Wunc,'c:');

%% Blad wzgledny
for i = 1 : length(Garray)
    sv = sigma(Garray(:,:,i) - Gnom, 1);
    if(sv(1) > 0.00001)
        sigma((Garray(:,:,i) - Gnom)/Gnom,'k:');
        hold on;
    else
        fprintf('Gnom skipped\n');
    end
end
sigma(Info.W1, 'b');

%%
%[W,Info] = ucover(Garray,Gnom,1);
lineStyles = {'y','g','b','r'};
for i = 2:5
    [W,Info] = ucover(Garray,Gnom,i);
    %[W,Info] = ucover(Gnom,Info,i,0);

    if type == 1
        sv = sigma(Info.W1, w);
        loglog(w,sv, lineStyles{i-1})
    else
        sv = sigma(Gnom + Info.W1*Gnom, w);
        loglog(w,sv, lineStyles{i-1})
    end
end

%% Multiplikatywny model niepewnosci

%% Zakres wartosci elementow macierzy stanu
clear

wing = WingFlutter;
plane = PlaneParams(wing.params);

% Model nominalny
wing.U0 = wing.getFlutterSpeed;
Gnom = wing.getLinearModel;

A = zeros(size(Gnom.a,1), size(Gnom.a,2), 3*3*3);
B = zeros(size(Gnom.b,1), size(Gnom.b,2), 3*3*3);
C = zeros(size(Gnom.c,1), size(Gnom.c,2), 3*3*3);
D = zeros(size(Gnom.d,1), size(Gnom.d,2), 3*3*3);

n = 1;
%for H = [0 5000 10000]
    %wing.atmosphere.h = H;
    for fuel = [0, 0.5, 1]
        plane.fuelLevel = fuel;
        for speed = [50 90 130]
            wing.U0 = speed;
            sys = wing.getLinearModel;
            A(:,:,n) = sys.a;
            B(:,:,n) = sys.b;
            C(:,:,n) = sys.c;
            D(:,:,n) = sys.d;
            n = n+1;
        end
    end
%end

%%
Anom = (max(A,[],3) + min(A,[],3))/2;
Adelta = (max(A,[],3) - min(A,[],3))./Anom;

