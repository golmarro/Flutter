function [Garray Gnom] = modelFamily(type)
    %% Zwraca rodzine modeli Garray oraz rysuje wykres wartoœci osobliwej 
    %  (SV) dla rodziny modeli + dla modelu model nominalnego

    wing = WingFlutter;
    plane = PlaneParams(wing.params);

    % Sygnaly wejsciowe / wyjsciowe brane pod uwage (istotne z punktu widzenia 
    % zalozen projektowych)
    inSignal = [1 2];           % delta_c turb
    outSignal = [1 2 8 9];      % h theta h_dd theta_dd

    % Typ wykresu
    % 0 none
    % 1 - (Gi - Gn), Wu
    % 2 -  Gi, Gn, (I + Wu)Gn

    % Model nominalny
    Vk = wing.getFlutterSpeed;
    wing.U0 = Vk;
    wing.params.fuelLevel = 0.5;
    wing.atmosphere.h = 5500;
    Gnom = wing.getLinearModel;
    Gnom = Gnom(outSignal,inSignal);

    % Oszacowanie zakresu czestosci w
    [sv_nom w] = sigma(Gnom);
    % Najpier rysujemy model nominalny
    if(type == 2)
        sv = sigma(Gnom, w);
        loglog(w, sv(1,:), 'r', 'LineWidth',2);
    end
    hold on;

    % Rodzina modeli Gi
    clear Garray
    %Garray = stack(1,Gnom);



    % Przygotowanie obrazu
    % figure; 
    % if(type == 1)
    %     loglog(1,1);
    % else
    %     loglog(w, sv_nom(1,:), 'LineWidth', 2, 'Color','r');
    % end
    % hold on;

    %Hrange = [0 5500 11000];
    Hrange = 5500;
    %fuelRange = [0, 0.5, 1];
    fuelRange = [0:0.2:1];
    speedRange = [Vk*0.8 : 0.1*Vk : 1.2*Vk];
    %speedRange = wing.U0;

    for H = Hrange
        wing.atmosphere.h = H;
        for fuel = fuelRange
            plane.fuelLevel = fuel;
            for speed = speedRange
                % Pomin Gnom 
    %             if H == 5500 && fuel == 0.5 && speed == Vk
    %                 continue
    %             end

                wing.U0 = speed;
                sys = wing.getLinearModel;
                sys = sys(outSignal,inSignal);

                if type == 1
                    sv = sigma(sys - Gnom, w);
                    loglog(w,sv(1,:)./sv_nom(1,:));
                elseif type == 2
                    sv = sigma(sys, w);
                    loglog(w, sv(1,:), 'k:');
                    %sigma(sys, w);
                    hold on;
                end

                if ~exist('Garray','var')
                    Garray = stack(1,sys);
                else
                    Garray = stack(1,Garray,sys);
                end
            end
        end
    end
    if(type ~= 0)
        ylabel('\sigma_{max}(G)')
        xlabel('\omega [rad/s]')
        set(gca,'XLim',[4 1400])
        set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', [14 8]);
        set(gcf,'Units','centimeters','Position',[0, 0, 14, 8]);
    end

end