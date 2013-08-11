
clear
close_system('PlaneSim', 0);

%% Wczytanie modelu

try
    load PlaneSysNom
    fprintf('Nominal Plane system loaded from file\n')
catch me
    fprintf('Could not find Nominal Plane system, creating new one\n')
    simRunner = PlaneRunner('simRunner');
    plane = PlaneLtiModel;
    % Linearize for 
    sysArray = plane.linearizePrivate(90,0.5,5500,'PlaneSim',simRunner);
    Gnom = sysArray{1};
    
    sysArray = plane.linearizePrivate(90,0.5,5500,'PlaneStiffSim',simRunner);
    GnomStiff = sysArray{1};
    save PlaneSysNom Gnom GnomStiff
end

%% Funkcje wagowe
delta_max = 45*pi/180;
Wu = diag([delta_max delta_max delta_max delta_max]);

%
wpqr = tf(0.3*delta_max,[1/(3*2*pi)  1]);

% pasmo przenoszenia ukladu sterowania ~ 20 rad/s (18,8 Hz)
% MIL:  cutoff frequency (gain = 1) mniejsze od 30 rad/s, zeby nie wzbudzac
% elastycznych postaci ruchu

% Maly blad ponizej pulsacji odciecia, pulsacja odciecia = 20 rad/s
we = tf([1/2 1]./10,[1/200 1]);
We = append(we, we, we);

%% Konstruowanie wa¿onego modelu generycznego / modelu do syntezy  --- Pw

% Pomijamy turbulencje
P = Gnom(end-2:end,1:4);

%%
systemnames = 'P We Wu';
inputvar = '[p_ref;q_ref;r_ref;r_ail;l_ail;r_stab;l_stab]';
outputvar = '[We;Wu;p_ref-P(1);q_ref-P(2);r_ref-P(3)]';
input_to_P = '[r_ail;l_ail;r_stab;l_stab]';
input_to_We = '[p_ref-P(1);q_ref-P(2);r_ref-P(3)]';
input_to_Wu = '[r_ail;l_ail;r_stab;l_stab]';
Pw = sysic;

%% Synthesis

[Khinf, CL, gamma, info] = hinfsyn(Pw, 2, 1, 'display', 'on');


%---------------------------------------------------------------------
%% Ograniczamy sie do p, q
P = Gnom(end-2:end-1,1:4);
We = append(we, we);

%%
systemnames = 'P We Wu';
inputvar = '[p_ref;q_ref;r_ail;l_ail;r_stab;l_stab]';
outputvar = '[We;Wu;p_ref-P(1);q_ref-P(2)]';
input_to_P = '[r_ail;l_ail;r_stab;l_stab]';
input_to_We = '[p_ref-P(1);q_ref-P(2)]';
input_to_Wu = '[r_ail;l_ail;r_stab;l_stab]';
Pw = sysic;

%% Synthesis

[Khinf, CL, gamma, info] = hinfsyn(Pw, 2, 1, 'display', 'on');
SvdAnalysis(CL)


%% ---------------------------------------------------------- Stiff
P = GnomStiff(7:8,:);
We = append(we, we);
Wpqr = append(wpqr, wpqr);
Wpqr = [0.2 0; 0 0.2];

%%
systemnames = 'P We Wu Wpqr';
inputvar = '[p_ref;q_ref;r_ail;l_ail;r_stab;l_stab]';
outputvar = '[We;Wu;Wpqr(1)-P(1);Wpqr(2)-P(2)]';
input_to_P = '[r_ail;l_ail;r_stab;l_stab]';
input_to_We = '[Wpqr(1)-P(1);Wpqr(2)-P(2)]';
input_to_Wpqr = '[p_ref;q_ref]';
input_to_Wu = '[r_ail;l_ail;r_stab;l_stab]';
Pw = sysic;

%% Synthesis

[Khinf, CL, gamma, info] = hinfsyn(Pw, 2, 1, 'display', 'on');
%%
SvdAnalysis(CL)