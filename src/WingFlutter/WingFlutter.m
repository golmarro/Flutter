classdef WingFlutter < handle
    %WingFlutter Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Constant)
        g = 9.81;           % m/s2
    end
    properties(GetAccess = 'public', SetAccess = 'public')
        U0 = 50;           % speed m/s
        qOverride = Inf;    % To set dyn. pressure directly
        isGravity = 'on'
        accelOutputs = 'off'
        AeroForces = 'on'
        
        inputSignal = WingFlutter.stepSignal() % Input signal function
        params;             % Wing structural and aero params
        atmosphere;         % Flight conditions (altitude, density etc.)
        
        simStyle = 'k'
        lineFormat = 'b'
        
        actuatorModel = 'on'
    end
    
    properties(Dependent)
        % Macierzowa postac parametrow strukturalnych
        Ms, Ks, Ds
        % Opis sil aero
        Ma, Ka, Da, QT
        % Sterowanie
        B0, B1, B2
        % Wypadkowe macierze
        M, D, K
        % Wplyw grawitacji
        Mg
        % Cisnienie dynamiczne
        q       % [N/m2]
        % Silownik
        actuator
    end
    
    methods
        function this = WingFlutter(params, atmosphere)
            if ~exist('params','var')
                params = WingParams();
            end
            this.params = params;
            if ~exist('atmosphere','var')
                atmosphere = Atmosphere;
            end
            this.atmosphere = atmosphere;
        end
        
        % --------------------------------------- [getters / setters]
        
        function q = get.q(this)
            if strcmp(this.AeroForces, 'off')
                q = 0;
            else
                q = 0.5 * this.atmosphere.rho * this.U0^2;
            end
        end
        
        function Ms = get.Ms(this)
            Ms = [this.params.m this.params.shtheta; this.params.shtheta this.params.Itheta];
        end
        
        function Ks = get.Ks(this)
            Ks = [this.params.Kh 0; 0 this.params.Ktheta];
        end
        
        function Ds = get.Ds(this)
            % Here we use point mass, not the reduced
            tmp = [this.params.mass this.params.shtheta; this.params.shtheta this.params.Itheta];
            Ds = tmp*[2*this.params.zetah*this.params.omegah 0; 0 2*this.params.zetatheta*this.params.omegatheta]; %TODO
        end
        
        function Ma = get.Ma(this)
            Ma = this.q*this.params.S*this.params.c/(2*this.U0^2) * ...
                [-this.params.CLalphadot, -this.params.l*this.params.CLalphadot; 
                 this.params.c*this.params.CMalphadot, this.params.c*this.params.l*this.params.CMalphadot];
        end
        
        function Da = get.Da(this)
            Da = this.q*this.params.S/this.U0 *... 
                [-this.params.CLalpha ,  -this.params.l*this.params.CLalpha  - this.params.c/2 * (this.params.CLalphadot+this.params.CLq);
                 this.params.c*this.params.CMalpha,  this.params.c*this.params.l*this.params.CMalpha + this.params.c*this.params.c/2*(this.params.CMalphadot+this.params.CMq)];
        end
        
        function Ka = get.Ka(this)
            Ka = this.q*this.params.S*[0 -this.params.CLalpha; 0  this.params.c*this.params.CMalpha];
        end
        
        function QT = get.QT(this)
            QT = this.q*this.params.S*[-this.params.CLalpha; this.params.c*this.params.CMalpha];
        end
        
        function B0 = get.B0(this)
            B0 = this.q*this.params.S*[-this.params.CLdelta; this.params.c*this.params.CMdelta];
        end
        
        function M = get.M(this)
            M = this.Ms - this.Ma;
        end
        
        function D = get.D(this)
            D = this.Ds - this.Da;
        end
        
        function K = get.K(this)
            K = this.Ks - this.Ka;
        end
        
        function set.isGravity(this, is)
            assert(strcmp(is,'on') || strcmp(is,'off'))
            this.isGravity = is;
        end
        
        function Mg = get.Mg(this)
            if strcmp(this.isGravity, 'on')
                Mg = [this.params.mass * this.params.Ycg / this.params.Yac; this.params.shtheta];
            else
                Mg = [0; 0];
            end
        end
        
        function model = getModelSS(this)
            %        x                    v
            A = [  zeros(2)      ,      eye(2)     ;
                 -this.M\this.K  ,  -this.M\this.D ];
            B = [zeros(2,1)      ;  this.M\this.B0 ];
            
            C = eye(4);
            D = zeros(4,1);
            
            if strcmp(this.accelOutputs, 'on');
                C = [C; 
                    0 0 -1/this.g -this.params.Dleo/this.g; 
                    0 0 -1/this.g  this.params.Dteo/this.g];
                % TODO tutaj w D brakuje wplywu deltadotdot na przysp.
                D = zeros(6,1);
            end
            
            model = ss(A, B, C, D);
            states          = ['h [ft]           ';
                               'theta [rad]      ';
                               'h_dot [ft/s]     ';
                               'theta_dot [rad/s]'];
            if strcmp(this.actuatorModel, 'on')
                model = series(this.actuator,model, 1, 1);
                model.StateName = [states;
                               'actuator state1  ';
                               'actuator state2  '];
            else
                model.StateName = states;
            end
            
            model.InputName = 'delta[rad]';
            
            if strcmp(this.accelOutputs, 'on');
                model.OutputName = [states;
                               'acc leading [g]  ';
                               'acc trailing [g] '];
            else
                model.OutputName = states;
            end
        end
        
        function model = getModelMat(this)
            delta = @this.inputSignal;
            function ydot = Model(t, y)
                ydot = zeros(4,1);
                ydot(1:2) = y(3:4); % hdot thetadot 
                
                ydot(3:4) = (this.M)\(this.Mg*this.g*cos(this.params.alpha0)...
                    + this.QT*this.params.alpha0 + this.B0*delta(t)...
                    - this.K*y(1:2) - this.D*y(3:4));
            end
            model = @Model;
        end
        
        function showModes(this)
            sys = this.getModelSS();
            [V E] = eig(sys.A);
            for i = 1:size(V,1)
                mag = abs(V(:,i));
                phase = angle(V(:,i));
                freq = abs(E(i,i));
                d = -cos(angle(E(i,i)));
                fprintf('freq: %f [rad/s] damp: %f\n',freq, d);
                fprintf('                 %f %f %f %f\n',mag);
                fprintf('                 %f %f %f %f\n',floor(phase*180/pi));
            end
        end
        
        function state = trim(this, alph0, delta)
            if exist('alph0', 'var')
                this.params.alpha0 = alph0;
            end
            if ~exist('delta', 'var')
                delta = 0;
            end
%             state = this.K\(this.Q0e + this.QT*alph0...
%                 + this.Mg*this.g*cos(alph0) + this.B0*delta);
            state = this.K\(           this.QT*alph0...
                + this.Mg*this.g*cos(alph0) + this.B0*delta);
        end
        
        function alpha0 = trimLift(this, desiredLift)
            % initial guess
            a0 = inv(this.QT(1)) * desiredLift;
            
            function deltaLift = fun(alpha0)
                state = this.trim(alpha0, 0);
                forces = this.Ka * state + this.QT * alpha0;
                deltaLift = forces(1) - desiredLift;
            end
            
            alpha0 = fzero(@fun, a0);
        end
        
        function alpha0 = trimLift2(this, lift)
            alpha0 = pinv(this.QT) * [lift; 0];
            this.params.alpha0 = alpha0;
        end
        
        function alpha0 = trimForce(this, force)
            % initial guess
            a0 = inv(this.QT(1)) * force;
            h_desired = force/this.params.Kh;
            function h = fun(alpha0)
                state = this.trim(alpha0, 0);
                h = state(1) - h_desired;
            end
            
            alpha0 = fzero(@fun, a0);
        end
        
        function compareModels(this, models, stopTime)
            if ~exist('stopTime','var')
                stopTime = 4;
            end
            figure
            subplot(2,1,1); hold on; ylabel('h')
            subplot(2,1,2); hold on; ylabel('theta')
            
            format = {'k','r:','g.','y'};
            for i = 1:length(models)
                [t y] = ode45(models{1}, [0 stopTime], [0 0 0 0]);
                subplot(2,1,1)
                plot(t, y(:,1),format{i})
                subplot(2,1,2)
                plot(t, y(:,2)*180/pi,format{i})
            end
        end
        
        function [t y] = sim(this, stopTime, state)
            if ~exist('stopTime','var')
                stopTime = 10;
            end
            if ~exist('state','var')
                state = [0 0 0 0];
            end
            [t y] = ode45(this.getModelMat(), [0 stopTime], state);
            if nargout == 0
                subplot(2,1,1); hold on;
                ylabel('h [ft]');
                plot(t, y(:,1), this.simStyle)
                subplot(2,1,2); hold on; 
                ylabel('theta [deg]');
                plot(t, y(:,2)*180/pi, this.simStyle)
            end
        end
        
        function setInputSignal(this, type)
            type = lower(type);
            if strcmp(type, 'const')
                this.inputSignal = this.constSignal();
            elseif strcmp(type, 'step')
                this.inputSignal = this.stepSignal();
            end
        end
        
        function compareMass(this, stopTime)
            if ~exist('stopTime','var')
                stopTime = 20;
            end
            this.M = this.Ms;
            model1 = this.getModelMat();
            % TODO
%             this.M = this.Ms - this.Ma;
%             model2 = this.getModelMat();
            this.compareModels({model1, model2}, stopTime)
            legend('Only struct mass', 'Full model');
        end
        
        function Uf = SpeedRootLocus(this, U, format)
            if ~exist('U', 'var')
                U = 100:50:400;
            end
            if ~exist('format', 'var')
                format = 'b.';
            end
            Uf = Inf;
            locus = zeros(length(U), 4);
            for i = 1:length(U)
                this.U0 = U(i);
                % this.q = (rho*this.U0^2)/2;
                sys = this.getModelSS();
                [V E] = eig(sys.a);
                % Check if unstable
                if sum(real(diag(E)) > 0) > 0
                    Uf = min(Uf, U(i));
                end
                locus(i,:) = diag(E)';
            end
            plot(locus(:,1), format)
            hold on;
            plot(locus(:,2), format)
            plot(locus(:,3), format)
            plot(locus(:,4), format)
            % First speed
            plot(locus(1,:), 'ro')
            % Last speed
            plot(locus(end,:), 'rx')
            grid on
            if Uf ~= Inf
                fprintf('Flutter for U = %f [km/h], M= %f, alt = %f [m]\n',...
                    Uf*3.6, Uf/this.atmosphere.c, this.atmosphere.h);
            end
        end
        
        function isStab = isStable(this)
            isStab = true;
            sys = this.getModelSS();
            [~, E] = eig(sys.a);
            % Check if unstable
            if sum(real(diag(E)) > 0) > 0
                isStab = false;
            end
        end
        
        function Uf = getFlutterSpeed(this, left, right)
            if ~exist('left','var')
                left = 10;
            end
            if ~exist('right','var')
                right = this.atmosphere.c*2;
            end
            this.U0 = 10;
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
                this.atmosphere.h = alt(i);
                Uf(i) = this.getFlutterSpeed(lastU);
                lastU = Uf(i);
            end
            plot(Uf*3.6, alt, this.lineFormat);
            ylabel('Altitude [m]');
            xlabel('Flutter speed [km/h]')
            title('Flutter speed versus altitude');
        end
        
        function val = get.actuator(this)
            k = 1.02;           % [dg/deg]
            zeta = 0.56;        % [-]
            omega = 165.3;      % [rad/s]
            val = tf(k*omega^2, [1 2*zeta*omega omega^2]);
            
            if strcmp(this.actuatorModel, 'off')
                val = tf(1, 1);
            end
        end
        
        function disp(this)
            fprintf('WingFlutter model for:\n')
            fprintf('\tspeed: \t\t%f [km/h]\n', this.U0*3.6);
            fprintf('\tMach: \t\t%f\n', this.U0/this.atmosphere.c);
            fprintf('\tdyn. pressure: \t%f [Pa]\n', this.q);
            fprintf('\talpha0: \t%f\n', this.params.alpha0);
        end
    end
    
    methods(Static)
        function fun = stepSignal(stepTime, stepValue)
            if ~exist('stepTime','var')
                stepTime = 1;
            end
            if ~exist('stepValue','var')
                stepValue = 5*pi/180;
            end
            function s = signal(t)
                s = 0;
                if t > stepTime
                    s = stepValue;
                end
            end
            fun = @signal;
        end
        
        function fun = constSignal(value)
            if ~exist('value','var')
                value = 0;
            end
            function s = signal(t)
                s = 0;
            end
            fun = @signal;
        end
    end
    
end

