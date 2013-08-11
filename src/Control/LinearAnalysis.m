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
        out_h_dd        = 8;
        out_theta_dd    = 9;
        % additionally in closed-loop
        out_delta_c     = 10;
        
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
                B = this.OL.B(:,[1 1 2]);
                C = [this.OL.C; zeros(1, size(this.OL.C,2))];
                D = [this.OL.D(:,[1 1 2]); 1 0 0];
                sys = ss(this.OL.a, B, C, D);
%                 C = [this.OL.C; zeros(1, size(this.OL.C,2))];
%                 D = [this.OL.D; 1 0];
%                 sys = ss(this.OL.a, this.OL.b, C, D);
                sys.InputName = {this.OL.InputName{1}, 'delta_p', 'turb'};
%                 sys.InputName = this.OL.InputName;
                sys.OutputName = [this.OL.OutputName;
                                  'delta_c [rad]    '];
                
                
                % val = feedback(sys, this.K, 1, 3:4);
                if(size(this.K,2) == 4)
                    val = feedback(sys, this.K, 1, ...
                        [this.out_h_dot this.out_theta_dot this.out_h_dd this.out_theta_dd]);
                    val = val(:,[2 3]);
                else
                    val = feedback(sys, this.K, 'name');
                    val = val(:,[2 3]);
                end
                
                val.InputName = ['delta_p[rad]';
                                 'turb in     '];
                val.OutputName = ['h [ft]           ';
                                  'theta [rad]      ';
                                  'h_dot [ft/s]     ';
                                  'theta_dot [rad/s]';
                                  'delta [rad]      ';
                                  'w [m/s]          ';
                                  'w dot [m/s]      ';
                                  'h_dd [m/s2]      ';
                                  'theta_dd [rad/s2 ';
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
        function set.lineFormat(this, val)
            this.lineColor = val(1);
            if length(val) > 1
                this.lineStyle = val(2:end);
            else
                this.lineStyle = '-';
            end
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
        
        function maxDeltaAnalysis(this)
            Ufcl = this.Ufcl();
            Ufol = this.Ufol();
            U = (0.8*Ufol/Ufcl : 0.02 : 1) * Ufcl;
            Wdelta = tf(0.7,[1/(3*2*pi)  1]);
            maxDelta_c = zeros(size(U));
            
            for i = 1:length(U)
                this.U0 = U(i);
                if this.isStable()
                    delta = step(this.CL(this.out_delta_c,this.in_delta_p) * Wdelta);
                    maxDelta_c(i) = max(abs(delta));
                else
                    maxDelta_c(i) = 0;
                end
            end
            maxDelta_c = maxDelta_c*180/pi;
            plot(U,maxDelta_c, this.lineFormat);
            hold on;
            xlabel('Predkosc lotu U_0 [m/s]')
            ylabel('max(|\delta_c(t)|) [deg]');
            title('');
            handle = plot([Ufol Ufol],[0 max(maxDelta_c)], 'k:');
            hasbehavior(handle,'legend',false);
            handle = plot([Ufcl Ufcl],[0 max(maxDelta_c)], 'r:');
            hasbehavior(handle,'legend',false);
            handle = plot([U(1) U(end)],[1 1]*this.deltaMax*180/pi, 'k:');
            hasbehavior(handle,'legend',false);
            handle = plot([U(1) U(end)],[1 1]*this.deltaMax*this.deltaPercent*180/pi, 'k:');
            hasbehavior(handle,'legend',false);
        end
        
        function [t, y, rms] = controlSignalAnalysis(this)
            % Analysis of control signal (delta_c) due to pilot signal on
            % delta (delta_p)
            % delta_p -> delta_c
            % this.resetDefaultConditions();
            [y, t] = step(this.CL(:,this.in_delta_p) * 0.7 * this.deltaMax, this.deltaStepTime);
            %y = y - 0.7 * this.deltaMax;
            delta = y(:,this.out_delta_c);
            delta_dot = (delta - [delta(1); delta(1:end-1)]) ./ (t - [t(2:end); t(end) + t(end) - t(end-1) ]);
            rms = this.rmsCalc([y delta_dot]);
            
            if nargout == 0
                subplot(2,2,1); hold on;
                plot(t,(y(:,this.out_delta_c) - 0.7*this.deltaMax)*180/pi, this.lineFormat);
                xlabel('t [s]');
                ylabel('\delta_c [deg]');
                title('Control signal');
                
                subplot(2,2,2); hold on;
                plot(t,delta_dot*180/pi, this.lineFormat);
                xlabel('t [s]');
                ylabel('dot \delta_c  [deg/s]');
                title('Control signal effort');

                subplot(2,2,3); hold on;
                plot(t, y(:,this.out_h), this.lineFormat); ylabel('h [m]'); xlabel('t [s]');

                subplot(2,2,4); hold on;
                plot(t, y(:,this.out_theta)*180/pi, this.lineFormat); ylabel('\theta [deg]'); xlabel('t [s]');
                
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
                [delta, t] = step(this.CL(this.out_delta_c,this.in_delta_p) * 0.7 * this.deltaMax, this.deltaStepTime);
                delta_dot = (delta - [delta(1); delta(1:end-1)]) ./ (t - [t(2:end); t(end) + t(end) - t(end-1) ]);
                deltaRms(i) = this.rmsCalc(delta_dot);
                maxDelta_c(i) = max(abs(delta));
            end
            
            subplot(2,1,1); hold on;
            plot(U,deltaRms, this.lineFormat);
            xlabel('Speed U_0 [m/s]')
            ylabel('RMS of dot \delta_c [rad]');
            title('Control effort due to pilot command on \delta_p (0.7 \delta_{max})');
            handle = plot([Ufol Ufol],[0 max(deltaRms)], 'k:');
            hasbehavior(handle,'legend',false);
            handle = plot([Ufcl Ufcl],[0 max(deltaRms)], 'r:');
            hasbehavior(handle,'legend',false);
            
            subplot(2,1,2); hold on;
            plot(U, maxDelta_c*180/pi, this.lineFormat);
            xlabel('Speed U_0 [m/s]')
            ylabel('max |\delta_c(t)| [deg]');
            title('Max aileron deflection due to pilot command on \delta_p (0.7 \delta_{max})');
            handle = plot([Ufol Ufol],[0 max(maxDelta_c*180/pi)], 'k:');
            hasbehavior(handle,'legend',false);
            handle = plot([Ufcl Ufcl],[0 max(maxDelta_c*180/pi)], 'r:');
            hasbehavior(handle,'legend',false);
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
                plot(t, y(:,this.out_w), this.lineFormat); ylabel('w [m/s]'); xlabel('t [s]');

                subplot(2,2,2); hold on;
                plot(t,y(:,this.out_delta_c)*180/pi, this.lineFormat); ylabel('\delta_c [deg]'); xlabel('t [s]');

                subplot(2,2,3); hold on;
                plot(t, y(:,this.out_h), this.lineFormat); ylabel('h [m]'); xlabel('t [s]');

                subplot(2,2,4); hold on;
                plot(t, y(:,this.out_theta)*180/pi, this.lineFormat); ylabel('\theta [deg]'); xlabel('t [s]');
                
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
                [delta, t] = lsim(this.CL(this.out_delta_c,this.in_turb), eta, t);
                delta_dot = (delta - [delta(1); delta(1:end-1)]) ./ (t - [t(2:end); t(end) + t(end) - t(end-1) ]);
                deltaRms(i) = this.rmsCalc(delta_dot);
            end
            
            plot(U, deltaRms, this.lineFormat);
            hold on;
            handle = plot([Ufol Ufol],[0 max(deltaRms)], 'k:');
            hasbehavior(handle,'legend',false);
            handle = plot([Ufcl Ufcl],[0 max(deltaRms)], 'r:');
            hasbehavior(handle,'legend',false);
            xlabel('Speed [m/s]'); ylabel('RMS of d \delta_c / dt signal [rad/s]')
            title('RMS of d \delta_c / dt signal due to turbulence for different speed');
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
        
        function [U Hinf] = hinfAnalysis(this, inSignal, outSignal)
            % Hinf in case of siso is trivial
            U = (0.8*this.Ufol/this.Ufcl : 0.02 : 1) * this.Ufcl;
            Hinf = zeros(size(U));
            for i = 1:length(U)
                this.U0 = U(i);
                [mag, ~] = bode(this.CL(outSignal,inSignal));
                Hinf(i) = max(mag);
            end
            
            if nargout == 0
                plot(U, Hinf, this.lineFormat);
                hold on;
            
                handle = plot([this.Ufol this.Ufol],[0 max(Hinf)], 'k:');
                hasbehavior(handle,'legend',false);
                handle = plot([this.Ufcl this.Ufcl],[0 max(Hinf)], 'r:');
                hasbehavior(handle,'legend',false);
%                 handle = plot([U(1) U(end)], [this.deltaMax this.deltaMax], 'k:');
%                 hasbehavior(handle,'legend',false);
%                 handle = plot([U(1) U(end)], [this.deltaMax this.deltaMax]*this.deltaPercent, 'k:');
%                 hasbehavior(handle,'legend',false);
                xlabel('Speed [m/s]');
                %title('H_\infty norm of \delta_c signal due to turbulence for different speeds');
            end
        end
        
        function turbulenceHinfAnalysis(this)
            % Calculate ||G turb delta_c || inf 
            % = sup (over omega) |G turb delta_c |
            this.resetDefaultConditions();
            Ufcl = this.Ufcl();
            Ufol = this.Ufol();
            
            %[U Hinf] = this.hinfAnalysis(this.in_turb, this.out_delta_c);
            U = (0.8*this.Ufol/this.Ufcl : 0.02 : 1) * this.Ufcl;
            Hinf = zeros(size(U));
            maxDelta = zeros(size(U));
            maxDelta2 = zeros(size(U));
            for i = 1:length(U)
                this.U0 = U(i);
                [mag, ~] = bode(this.CL(this.out_delta_c,this.in_turb));
                Hinf(i) = max(mag);
                [delta_c ~] = step(this.CL(this.out_delta_c,this.in_turb));
                maxDelta(i) = max(abs(delta_c));
                [t, eta] = this.getTurbSampleSignal();
                delta_c = lsim(this.CL(this.out_delta_c,this.in_turb), eta, t);
                maxDelta2(i) = max(abs(delta_c));
            end
            
            plot(U, Hinf, this.lineFormat);
            hold on;
            plot(U,maxDelta,'b--');
            plot(U,maxDelta2,'b:');
            legend('Hinf','step','random');
            handle = plot([Ufol Ufol],[0 max(Hinf)], 'k:');
            hasbehavior(handle,'legend',false);
            handle = plot([Ufcl Ufcl],[0 max(Hinf)], 'r:');
            hasbehavior(handle,'legend',false);
            handle = plot([U(1) U(end)], [this.deltaMax this.deltaMax], 'k:');
            hasbehavior(handle,'legend',false);
            handle = plot([U(1) U(end)], [this.deltaMax this.deltaMax]*this.deltaPercent, 'k:');
            hasbehavior(handle,'legend',false);
            xlabel('Speed [m/s]'); ylabel('||G_{\eta \delta}||_\infty [rad]')
            title('H_\infty norm of \delta_c signal due to turbulence for different speeds');
        end
        
        function controlHinfAnalysis(this)
            % Calculate ||G turb delta_c || inf 
            % = sup (over omega) |G turb delta_c |
            this.resetDefaultConditions();
            Ufcl = this.Ufcl();
            Ufol = this.Ufol();
            
            Wdelta = zpk([],-3*2*pi,0.7);
            U = (0.8*this.Ufol/this.Ufcl : 0.02 : 1) * this.Ufcl;
            Hinf = zeros(size(U));
            maxDelta = zeros(size(U));
            maxDelta2 = zeros(size(U));
            for i = 1:length(U)
                this.U0 = U(i);
                [mag, ~] = bode(this.CL(this.out_delta_c,this.in_delta_p)*Wdelta);
                Hinf(i) = max(mag);
                [delta_c ~] = step(this.CL(this.out_delta_c,this.in_delta_p)*Wdelta);
                maxDelta(i) = max(abs(delta_c));
                [delta_c ~] = step(this.CL(this.out_delta_c,this.in_delta_p)*0.7);
                maxDelta2(i) = max(abs(delta_c));
            end
            
            plot(U, Hinf, this.lineFormat);
            hold on;
            plot(U,maxDelta,'b--');
            plot(U,maxDelta2,'b:');
            
            legend('Hinf','max |\delta_c(t)| (filtr)', 'max |\delta_c(t)|');
            
            handle = plot([Ufol Ufol],[0 max(Hinf)], 'k:');
            hasbehavior(handle,'legend',false);
            handle = plot([Ufcl Ufcl],[0 max(Hinf)], 'r:');
            hasbehavior(handle,'legend',false);
            handle = plot([U(1) U(end)], [this.deltaMax this.deltaMax], 'k:');
            hasbehavior(handle,'legend',false);
            handle = plot([U(1) U(end)], [this.deltaMax this.deltaMax]*this.deltaPercent, 'k:');
            hasbehavior(handle,'legend',false);
            xlabel('Speed [m/s]');
            title('Analiza sygnalu sterowania wzgledem komend pilota dla roznych predkosci');
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
            cl = feedback(this.OL, this.K, 'name');
            [~, E] = eig(cl.a);
            %[~, E] = eig(this.CL.a);
            % Check if unstable
            if sum(real(diag(E)) > 0) > 0
                isStab = false;
            end
        end
        
        % Speed where delta_c exceeds max value
        % || G turbulence -> delta_c || > 1
        function isStab = isRealStable(this)
            isStab = true;
            [mag, ~] = bode(this.CL(this.out_delta_c, this.in_turb));
            % Check if unstable
            if max(mag) > 1 || ~this.isStable()
                isStab = false;
            end
        end
        
        % Speed where delta_c exceeds max value
        % || G turbulence -> delta_c || > 1
        function isStab = isManouverable(this)
            isStab = true;
            %[mag, ~] = bode(0.7*this.CL(this.out_delta_c, this.in_delta_p));
            % Check if unstable
            %Wdelta = zpk([],-3*2*pi,0.7);
            Wdelta = tf(0.7,[1/(3*2*pi)  1]);
            delta = step(this.CL(this.out_delta_c, this.in_delta_p)*Wdelta);
            if max(abs(delta)) > this.deltaMax*this.deltaPercent
                isStab = false;
            end
%             if max(mag) > this.deltaMax*this.deltaPercent || ~this.isStable()
%                 isStab = false;
%             end
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
                left = 40;
            end
            if ~exist('right','var')
                right = this.wing.atmosphere.c*2;
            end
            this.U0 = left;
            %assert(this.isStable());
            if ~this.isStable()
                fprintf('CL unstable for: U = %f, fuel = %f, h = %f\n',...
                    this.U0, this.wing.params.fuelLevel, this.wing.atmosphere.h);
                Uf = -1;
                return;
            end
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
        
        function [Ufcl fuel] = flutterSpeedVersusFuel(this, fuel)
            if ~exist('fuel','var')
                fuel = 0:0.1:1;
            end
            Ufcl = zeros(1, length(fuel));
            %lastU = 30;
            for i = 1:length(fuel)
                this.wing.params.fuelLevel = fuel(i);
                Ufcl(i) = this.getFlutterSpeed(30);
                %lastU = Uf(i);
            end
            plot(Ufcl*3.6, fuel*100, this.lineFormat);
            ylabel('Fuel level [%]');
            xlabel('Flutter speed [km/h]');
            title('Flutter speed versus altitude');
        end
        
        function Uf = getManouverSpeed(this, left, right)
            if ~exist('left','var')
                left = 10;
            end
            if ~exist('right','var')
                right = this.wing.atmosphere.c*2;
            end
            this.U0 = left;
            % assert(this.isManouverable());
            if ~this.isManouverable()
                fprintf('CL unmanouverable for: U = %f, fuel = %f, h = %f\n',...
                    this.U0, this.wing.params.fuelLevel, this.wing.atmosphere.h);
                Uf = -1;
                return;
            end
            this.U0 = right;
            assert(~this.isManouverable());
            
            while right-left > 1
                new = (right+left)/2;
                this.U0 = new;
                if(this.isManouverable())
                    left = new;
                else
                    right = new;
                end
            end
            
            Uf = (right+left)/2;
        end
        
        function [Ufcl fuel] = manouverSpeedVersusFuel(this, fuel)
            if ~exist('fuel','var')
                fuel = 0:0.1:1;
            end
            Ufcl = zeros(1, length(fuel));
            %lastU = 30;
            for i = 1:length(fuel)
                this.wing.params.fuelLevel = fuel(i);
                Ufcl(i) = this.getManouverSpeed(30);
                %lastU = Uf(i);
            end
            plot(Ufcl*3.6, fuel*100, this.lineFormat);
            ylabel('Fuel level [%]');
            xlabel('Speed [km/h]');
%             title('Flutter speed versus altitude');
        end
        
        function Uf = getRealFlutterSpeed(this, left, right)
            % Speed where delta_c exceeds max value
            % || G turbulence -> delta_c || > 1
            if ~exist('left','var')
                left = 10;
            end
            if ~exist('right','var')
                right = this.wing.atmosphere.c*2;
            end
            this.U0 = left;
            assert(this.isRealStable());
            this.U0 = right;
            assert(~this.isRealStable());
            
            while right-left > 1
                new = (right+left)/2;
                this.U0 = new;
                if(this.isRealStable())
                    left = new;
                else
                    right = new;
                end
            end
            
            Uf = (right+left)/2;
        end
        
        function [Ufcl fuel] = realFlutterSpeedVersusFuel(this, fuel)
            if ~exist('fuel','var')
                fuel = 0:0.1:1;
            end
            Ufcl = zeros(1, length(fuel));
            %lastU = 30;
            for i = 1:length(fuel)
                this.wing.params.fuelLevel = fuel(i);
                Ufcl(i) = this.getRealFlutterSpeed(30);
                %lastU = Uf(i);
            end
            plot(Ufcl*3.6, fuel*100, this.lineFormat);
            ylabel('Fuel level [%]');
            xlabel('Speed [km/h]');
%             title('Flutter speed versus altitude');
        end
        
        function robustStabilityAnalysis(this, alt, fuel)
            if ~exist('alt','var')
                %alt = 0:1000:11000;
                alt = [0 5500 11000];
            end
            if ~exist('fuel','var')
                %fuel = 0:0.1:1;
                fuel = [0 0.5 1];
            end
            
            styles = {'r','r--','r:','g','g--','g:','b','b--','b:'};
            for i = 1:length(alt)
                this.wing.atmosphere.h = alt(i);
                this.lineFormat = styles{i};
                this.flutterSpeedVersusFuel(fuel);
                hold on;
            end
        end
        
        function robustnessAnalysis(this, alt, fuel)
            if ~exist('alt','var')
                %alt = 0:1000:11000;
                %alt = [0 5500 11000];
                alt = 5500;
            end
            if ~exist('fuel','var')
                %fuel = 0:0.1:1;
                fuel = [0 0.5 1];
            end
            
            styles = {'r','r--','r:','g','g--','g:','b','b--','b:'};
            % ------------------------------------ stability (flutter
            % speed)
            for i = 1:length(alt)
                this.wing.atmosphere.h = alt(i);
                %this.lineFormat = styles{i};
                this.lineStyle = '-';
                this.flutterSpeedVersusFuel(fuel);
                hold on;
            end
            
            % ------------------------------------- manouver
            for i = 1:length(alt)
                this.wing.atmosphere.h = alt(i);
                %this.lineFormat = styles{i};
                this.lineStyle = '--';
                this.manouverSpeedVersusFuel(fuel);
                hold on;
            end
            
            % ------------------------------------- max stress
            for i = 1:length(alt)
                this.wing.atmosphere.h = alt(i);
                %this.lineFormat = styles{i};
                this.lineStyle = ':';
                this.realFlutterSpeedVersusFuel(fuel);
                hold on;
            end
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

