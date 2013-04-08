% Niepewnosc modelu skrzydla

%% Wykres warto�ci osobliwej (SV) dla rodziny modeli
clear
wing = WingFlutter;
wing.atmosphere.h = 5500;
plane = PlaneParams(wing.params);

% Sygnaly wejsciowe / wyjsciowe brane pod uwage (istotny z punktu widzenia zalozen
% projektowych)
%inSignal = [1 2];
%outSignal = [1 2 8 9];
inSignal = 2;       % delta_c
outSignal = 8;      % theta_ddot

% Typ wykresu
% 1 - (Gi - Gn), Wu
% 2 -  Gi, Gn, (I + Wu)Gn
type = 1;

% Model nominalny
Vk = wing.getFlutterSpeed;
wing.U0 = Vk * 1.4;
Gnom = wing.getLinearModel;
Gnom = Gnom(outSignal,inSignal);

% Rodzina modeli Gi
Garray = stack(1,Gnom);

[sv_nom w] = sigma(Gnom);

% Przygotowanie obrazu
figure; 
if(type == 1)
    loglog(1,1);
else
    loglog(w, sv_nom(1,:), 'LineWidth', 2, 'Color','r');
end
hold on;


for H = [0 5500 11000]
    wing.atmosphere.h = H;
    for fuel = [0, 0.5, 1]
        plane.fuelLevel = fuel;
        for speed = [Vk*0.6 : 0.4*Vk : 1.4*Vk]
            % Pomin Gnom 
            if H == 5500 && fuel == 0.5 && speed == 1.4*Vk
                continue
            end
            
            wing.U0 = speed;
            sys = wing.getLinearModel;
            sys = sys(outSignal,inSignal);
            
            sv = sigma(sys - Gnom, w);
            if type == 1
                loglog(w,sv(1,:)./sv_nom(1,:));
            else
                loglog(w, sv(1,:));
            end
                
            Garray = stack(1,Garray,sys);
        end
    end
end
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
