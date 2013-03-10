
%% Control signal analysis due to pilot command on delta_p - time domain
% Simplest possible controller delta_c = -theta_dot
clear
figure
K = [-1 0];
analysis = LinearAnalysis(K);
OLanalysis = LinearAnalysis([0 0]);
ol = analysis.OL;
cl = analysis.CL;

% Open-loop analysis
OLanalysis.lineColor = 'k';
OLanalysis.lineStyle = ':';
OLanalysis.resetDefaultConditions();
OLanalysis.controlSignalAnalysis();
Uol = OLanalysis.U0;

% Closed-loop analysis
analysis.resetDefaultConditions();
analysis.controlSignalAnalysis();
U1 = analysis.U0;

analysis.lineColor = 'r';
analysis.resetDefaultConditions();
Ufol = analysis.wing.getFlutterSpeed();
analysis.U0 = Ufol;
analysis.controlSignalAnalysis();
U2 = analysis.U0;

l0 = sprintf('open-loop   U = %.1f m/s (%.0f%% U_{fol})', Uol, Uol/Ufol * 100);
l1 = sprintf('closed-loop U = %.1f m/s (%.0f%% U_{fol})', U1, U1/Ufol * 100);
l2 = sprintf('closed-loop U = %.1f m/s (%.0f%% U_{fol})', U2, U2/Ufol * 100);
legend(l0,l1,l2);

%% RMS analysis of control signal due to pilot command on delta_p
clear
figure
K = [-1 0];
analysis = LinearAnalysis(K);
analysis.controlSignalRmsAnalysis();

%% RMS depends on simulation time when the signal converges to zero

% for t = 0.05:0.05:1
%     [y, ~] = step(0.7 * analysis.deltaMax * analysis.CL(8,1), t);
%     y = y - 0.7 * analysis.deltaMax;
%     rms = analysis.rmsCalc(y);
%     fprintf('RMS of delta signal due to 0.7 delta_max input: %f\n', rms);
% end

%% Turbulence analysis
clear
figure
K = [-1 0];

analysis = LinearAnalysis(K);
analysis.resetDefaultConditions();
analysis.turbulenceAnalysis();
U1 = analysis.U0;

analysis.lineColor = 'r';
analysis.resetDefaultConditions();
Uf = analysis.wing.getFlutterSpeed();
analysis.U0 = Uf;
analysis.turbulenceAnalysis();
U2 = analysis.U0;

l1 = sprintf('U = %.1f m/s (%.0f%% U_f)', U1, U1/Uf * 100);
l2 = sprintf('U = %.1f m/s (%.0f%% U_f)', U2, U2/Uf * 100);
legend(l1,l2);

%% Turbulence rms analysis - Tune
clear
K = [-1 0];
analysis = LinearAnalysis(K);
analysis.turbulenceRmsAnalysisTune(1);
analysis.turbulenceRmsAnalysisTune(2);
analysis.turbulenceRmsAnalysisTune(3);
analysis.turbulenceRmsAnalysisTune(4);

%% Turbulence rms analysis

clear
figure
K = [-1 0];

analysis = LinearAnalysis(K);

analysis.turbulenceRmsAnalysis

%% Full analysis
clear
K = [-1 0];
analysis = LinearAnalysis(K);

f1 = figure; hold on;
f2 = figure; hold on;
analysis.fullAnalysis(f1, f2);

K = [0 -1];
analysis = LinearAnalysis(K);
analysis.fullAnalysis(f1, f2);
