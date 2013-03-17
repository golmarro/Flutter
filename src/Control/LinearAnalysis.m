classdef LinearAnalysis < handle
    %LINEARANALYSIS Collect
    %   Detailed explanation goes here
    properties(Constant)
        refH = 5000;            % reference altitude 5000 m
        deltaMax = 45*pi/180;   % Max aileron delfection
        deltaPercent = 0.3      % Part of control signal designed to be used by flutter suppresion
        deltaStepTime = 0.5     % Common value for RMS of output signal comparison
        refSpeed = 80           % m/s (Flutter speed about 112 m/s)
        
        % indexes of signals in full model
        in_delta_p = 1;
        in_turb    = 2;
        
        out_h           = 1;
        out_theta       = 2;
        out_h_dot       = 3;
        out_theta_dot   = 4;
        out_delta       = 5;
        out_w           = 6;
        out_w_dot       = 7;
        % additionally in closed-loop
        out_delta_c     = 8;
        
    end
        
    
    properties
        systems
        
        K = []      % Controller
        
        fuelLevels = [0, 0.3, 1]
        
        wing
        plane
        lineStyle = '-'
        lineColor = 'b'
    end
    
    properties(Dependent)
        OL          % Open-loop system
        CL          % Closed-loop system
        U0
        Ufol
        Ufcl
        lineFormat
    end
    
    methods
        function this = LinearAnalysis(K)
            if exist('K', 'var')
                this.K = K;
            end
            this.wing = WingFlutter;
            this.plane = PlaneParams(this.wing.params);
        end
        
        % -------------------------------------------------- [getters / setters]
        function val = get.OL(this)
            val = this.wing.getLinearModel();
        end
        
        function val = get.CL(this)
            if ~isempty(this.K)
                % Feed delta_c to output for easier analysis
                C = [this.OL.C; zeros(1, size(this.OL.C,2))];
                D = [this.OL.D; 1 0];
                sys = ss(this.OL.a, this.OL.b, C, D);
                
                val = feedback(sys, this.K, 1, 3:4);
                
                val.InputName = ['delta_p[rad]';
                                 'turb in     '];
                val.OutputName = ['h [ft]           ';
                                  'theta [rad]      ';
                                  'h_dot [ft/s]     ';
                                  'theta_dot [rad/s]';
                                  'delta [rad]      ';
                                  'w [m/s]          ';
                                  'w dot [m/s]      ';
                                  'delta_c [rad]    '];
            else
                warning('RG:all','No controller specified yet K = []');
                val = [];
            end
        end
        
        function val = get.U0(this)
            val = this.wing.U0;
        end
        
        function set.U0(this, val)
            this.wing.U0 = val;
        end
        
        function val = get.Ufol(this)
            val = this.wing.getFlutterSpeed();
        end
        
        function val = get.Ufcl(this)
            val = this.getFlutterSpeed();
        end
        
        function val = get.lineFormat(this)
            val = [this.lineColor this.lineStyle];
        end
        
        % -------------------------------------------------- [ Analysis ] ---------
        
        function resetDefaultConditions(this)
            this.plane.reset();
            this.wing.atmosphere.h = this.refH;
            this.U0 = this.refSpeed;
        end
        
        function flutterSpeedAnalysis(this)
            figure;
            hold on;
            this.lineColor = 'b';
            this.stallSpeedVersusAlt();
            this.lineColor = 'k';
            this.flutterSpeedVersusAlt();
            this.lineColor = 'r';
            this.wing.lineFormat = this.lineFormat;
            this.wing.flutterSpeedVersusAlt();
            
            this.resetDefaultConditions();
            Uf1 = this.Ufol();
            Uf2 = this.Ufcl();
            fprintf('Flutter speed increased by %f%%\n', (Uf2/Uf1-1)*100);
        end
        
        function [t, y, rms] = controlSignalAnalysis(this)
            % Analysis of control signal (delta_c) due to pilot signal on
            % delta (delta_p)
            % delta_p -> delta_c
            % this.resetDefaultConditions();
            [y, t] = step(0.7 * this.deltaMax * this.CL(:,this.in_delta_p), this.deltaStepTime);
            %y = y - 0.7 * this.deltaMax;
            delta = y(:,this.out_delta_c);
            delta_dot = (delta - [delta(1); delta(1:end-1)]) ./ (t - [t(2:end); t(end) + t(end) - t(end-1) ]);
            rms = this.rmsCalc([y delta_dot]);
            
            if nargout == 0
                subplot(2,2,1); hold on;
                plot(t,(y(:,8) - 0.7*this.deltaMax)*180/pi, this.lineFormat);
                xlabel('t [s]');
                ylabel('\delta_c [deg]');
                title('Control signal');
                
                subplot(2,2,2); hold on;
                plot(t,delta_dot*180/pi, this.lineFormat);
                xlabel('t [s]');
                ylabel('dot \delta_c  [deg/s]');
                title('Control signal effort');

                subplot(2,2,3); hold on;
                plot(t, y(:,1), this.lineFormat); ylabel('h [m]'); xlabel('t [s]');

                subplot(2,2,4); hold on;
                plot(t, y(:,2)*180/pi, this.lineFormat); ylabel('\theta [deg]'); xlabel('t [s]');
                
                fprintf('RMS of delta_dot signal due to 0.7 delta_max input: %f [rad/s]\n', rms(end));
            end
        end
        
        function controlSignalRmsAnalysis(this)
            % Full speed range analysis of control signal (delta_c) due to pilot signal on
            % delta (delta_p)
            this.resetDefaultConditions();
            Ufcl = this.Ufcl();
            Ufol = this.Ufol();
            U = (0.8*Ufol/Ufcl : 0.02 : 1) * Ufcl;
            
            deltaRms = zeros(size(U));
            maxDelta_c = zeros(size(U));
            
            for i = 1:length(U)
                this.U0 = U(i);
                [~, y, rms] = this.controlSignalAnalysis();
                deltaRms(i) = rms(end);
                maxDelta_c(i) = max(abs(y(:,this.out_delta_c)));
            end
            
            subplot(2,1,1); hold on;
            plot(U,deltaRms, this.lineFormat);
            xlabel('Speed U_0 [m/s]')
            ylabel('RMS of dot \delta_c [rad]');
            title('Control effort due to pilot command on \delta_p (0.7 \delta_{max})');
            plot([Ufol Ufol],[0 max(deltaRms)], 'k:');
            plot([Ufcl Ufcl],[0 max(deltaRms)], 'r:');
            
            subplot(2,1,2); hold on;
            plot(U, maxDelta_c*180/pi, this.lineFormat);
            xlabel('Speed U_0 [m/s]')
            ylabel('max |\delta_c(t)| [deg]');
            title('Max aileron deflection due to pilo command on \delta_p (0.7 \delta_{max})');
            plot([Ufol Ufol],[0 max(maxDelta_c*180/pi)], 'k:');
            plot([Ufcl Ufcl],[0 max(maxDelta_c*180/pi)], 'r:');
        end
        
        function  [t, y, rms] = turbulenceAnalysis(this, finalTime)
            % Analysis for specific U0
            % turbulence -> delta_c
            % this.resetDefaultConditions();
            if ~exist('finalTime', 'var')
                finalTime = 4;
            end
            
            [t, eta] = this.getTurbSampleSignal(finalTime);
            y = lsim(this.CL(:,this.in_turb), eta, t);
            
            rms = this.rmsCalc(y(:,this.out_delta_c));
            
            if nargout == 0
                subplot(2,2,1); hold on;
                plot(t, y(:,6), this.lineFormat); ylabel('w [m/s]'); xlabel('t [s]');

                subplot(2,2,2); hold on;
                plot(t,y(:,8)*180/pi, this.lineFormat); ylabel('\delta_c [deg]'); xlabel('t [s]');

                subplot(2,2,3); hold on;
                plot(t, y(:,1), this.lineFormat); ylabel('h [m]'); xlabel('t [s]');

                subplot(2,2,4); hold on;
                plot(t, y(:,2)*180/pi, this.lineFormat); ylabel('\theta [deg]'); xlabel('t [s]');
                
                fprintf('RMS of delta signal due to turbulence input: %f\n', rms);
            end
        end
        
        function turbulenceRmsAnalysis(this)
            % Analysis for full speed range (0.8 Ufcl...Ufcl)
            this.resetDefaultConditions();
            Ufcl = this.Ufcl();
            Ufol = this.Ufol();
            [t, eta] = this.getTurbSampleSignal();
            
            U = (0.8*Ufol/Ufcl : 0.02 : 1) * Ufcl;
            deltaRms = zeros(size(U));
            for i = 1:length(U)
                this.U0 = U(i);
                y = lsim(this.CL(this.out_delta_c,this.in_turb), eta, t);
                deltaRms(i) = this.rmsCalc(y);
            end
            
            plot(U, deltaRms, this.lineFormat);
            hold on;
            plot([Ufol Ufol],[0 max(deltaRms)], 'k:');
            plot([Ufcl Ufcl],[0 max(deltaRms)], 'r:');
            xlabel('Speed [m/s]'); ylabel('RMS of \delta_c signal [rad/s]')
            title('RMS of \delta_c signal due to turbulence for different speed');
        end
        
        function turbulenceRmsAnalysisTune(this, finalTime)
            % Find the right finalTime for which rms result is quite const
            this.resetDefaultConditions();
            this.U0 = this.Ufol(); % Ufol
            
            for i = 1:20
                [t, eta] = this.getTurbRandomSignal(finalTime);
                y = lsim(this.CL(this.out_delta_c,this.in_turb), eta, t);
                deltaRms(i) = this.rmsCalc(y);
            end
            
            deltaRms = sort(deltaRms);
            %disp(deltaRms');
            war = this.rmsCalc(deltaRms - mean(deltaRms));
            DeltaMax = deltaRms(end) - deltaRms(1);
            fprintf('finalTime = %f, war: %f, max rozbieznosc: %f, wzgledna: %f\n',...
                finalTime, war, DeltaMax, DeltaMax / mean(deltaRms));
        end
        
        function turbulenceHinfAnalysis(this)
            % Calculate ||G turb delta_c || inf 
            % = sup (over omega) |G turb delta_c |
            this.resetDefaultConditions();
            Ufcl = this.Ufcl();
            Ufol = this.Ufol();
            
            U = (0.8*Ufol/Ufcl : 0.02 : 1) * Ufcl;
            delta_c_Hinf = zeros(size(U));
            for i = 1:length(U)
                this.U0 = U(i);
                [mag, ~] = bode(this.CL(this.out_delta_c,this.in_turb));
                delta_c_Hinf(i) = max(mag);
            end
            
            plot(U, delta_c_Hinf, this.lineFormat);
            hold on;
            plot([Ufol Ufol],[0 max(delta_c_Hinf)], 'k:');
            plot([Ufcl Ufcl],[0 max(delta_c_Hinf)], 'r:');
            plot([U(1) U(end)], [this.deltaMax this.deltaMax], 'k:');
            plot([U(1) U(end)], [this.deltaMax this.deltaMax]*this.deltaPercent, 'k:');
            xlabel('Speed [m/s]'); ylabel('||G_{\eta \delta}||_\infty [rad]')
            title('H_\infty norm of \delta_c signal due to turbulence for different speeds');
        end
        
        function fullAnalysis(this, f1, f2)
            this.resetDefaultConditions();
            
            this.U0 = this.Ufol;
            [~, ~, rmsTurbulence] = this.turbulenceAnalysis();
            
            this.U0 = this.Ufol;
            [~, y, rmsPilot] = this.controlSignalAnalysis();
            maxDelta_c = max(abs(y(:,this.out_delta_c)));
            
            if exist('f1', 'var')
                figure(f1);
            else
                figure; hold on;
            end
            this.controlSignalRmsAnalysis();
            
            if exist('f2', 'var')
                figure(f2);
            else
                figure; hold on;
            end
            this.turbulenceRmsAnalysis();
            
            fprintf('Ufcl/Ufol      turb -> RMS    delta_p -> RMS    delta_p -> max(delta)[deg]\n');
            fprintf('%.2f %%       %.3f          %.3f             %.2f\n',...
                (this.Ufcl/this.Ufol)*100, rmsTurbulence, rmsPilot(end), maxDelta_c*180/pi);
            fprintf('Response to Pilot step input:\n');
            fprintf('RMS(delta_c):   %f\n', rmsPilot(end));
            fprintf('RMS(h):         %f\n', rmsPilot(1));
            fprintf('RMS(theta):     %f\n', rmsPilot(2));
            fprintf('RMS(h_dot):     %f\n', rmsPilot(3));
            fprintf('RMS(theta_dot): %f\n', rmsPilot(4));
        end
        % -------------------------------------------------- [ Helpers ] ----
        
        function isStab = isStable(this)
            isStab = true;
            [~, E] = eig(this.CL.a);
            % Check if unstable
            if sum(real(diag(E)) > 0) > 0
                isStab = false;
            end
        end
             
        function stallSpeedVersusAlt(this,h)
            if ~exist('h','var')
                h = 0:100:11000;
            end
            Ustall = zeros(size(h));

            for i = 1:length(h)    % [m]
                this.wing.atmosphere.h = h(i);
                Ustall(i) = sqrt(2 * (this.plane.totalMass/2 * this.wing.g) ./...
                    (this.wing.atmosphere.rho*this.wing.params.S*this.wing.params.alphaMax*this.wing.params.CLalpha));
            end

            plot(Ustall*3.6, h, this.lineFormat);

            xlabel('U [km/h]');
            ylabel('Altitude [m]');
        end
        
        function Uf = getFlutterSpeed(this, left, right)
            if ~exist('left','var')
                left = 10;
            end
            if ~exist('right','var')
                right = this.wing.atmosphere.c*2;
            end
            this.U0 = left;
            assert(this.isStable());
            this.U0 = right;
            assert(~this.isStable());
            
            while right-left > 1
                new = (right+left)/2;
                this.U0 = new;
                if(this.isStable())
                    left = new;
                else
                    right = new;
                end
            end
            
            Uf = (right+left)/2;
        end
        
        function flutterSpeedVersusAlt(this, alt)
            if ~exist('alt','var')
                alt = 0:1000:11000;
            end
            Uf = zeros(1, length(alt));
            lastU = 30;
            for i = 1:length(alt)
                this.wing.atmosphere.h = alt(i);
                Uf(i) = this.getFlutterSpeed(lastU);
                lastU = Uf(i);
            end
            plot(Uf*3.6, alt, this.lineFormat);
            ylabel('Altitude [m]');
            xlabel('Flutter speed [km/h]');
            title('Flutter speed versus altitude');
        end
    end
    
    methods(Static)
        function rms = rmsCalc(y)
            % Returns rms for each vector (vertical)
            n = size(y,1);
            rms = sqrt(sum(y.*y,1) ./ n);
            %rms = norm(y)/sqrt(length(y));
        end
        
        function [t, eta] = getTurbRandomSignal(finalTime)
            t = 0:0.005:finalTime;
            eta = randn(size(t));
        end
        
        function [t, eta] = getTurbSampleSignal(finalTime)
            try
                load randTurb
            catch me
                t = 0:0.005:4;
                eta = randn(size(t));
                save randTurb t eta
            end
            if exist('finalTime', 'var')
                if finalTime < t(end)
                    i = floor(length(t) * finalTime/t(end));
                    t = t(1:i);
                    eta = eta(1:i);
                elseif finalTime > t(end)
                    warning('Max time = 4');
                end
            end
        end
    end
    
end

