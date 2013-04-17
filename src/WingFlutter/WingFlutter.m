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
        turbulenceInputSignal = WingFlutter.constSignal(0) % Turbulence input signal
        params;             % Wing structural and aero params
        atmosphere;         % Flight conditions (altitude, density etc.)
        
        simStyle = 'k'
        lineFormat = 'b'
        
        actuatorModel = 'off'
    end
    
    properties(Dependent)
        % Macierzowa postac parametrow strukturalnych
        Ms, Ks, Ds
        % Opis sil aero
        Ma, Ka, Da, QT, Qw  % Qw - turbulence
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
        
        function Qw = get.Qw(this)
            % [h_dot; theta_dot] = Qw * [wg; wg_dot]
            Qw = this.q*this.params.S/this.U0 * ...
                 [-this.params.CLalpha              , -this.params.c/(2*this.U0) * this.params.CLalphadot;
                   this.params.c*this.params.CMalpha, this.params.c^2/(2*this.U0)* this.params.CMalphadot];
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
        
        % Full model: wing, actuator, turbulence
        function model = getLinearModel(this)
            %        x                    v
            A = [  zeros(2)      ,      eye(2)     ;
                 -this.M\this.K  ,  -this.M\this.D ];
            B = [zeros(2,1)      ;  this.M\this.B0 ];
            
            C = eye(4);
            
            act = this.actuator;
            turb = this.params.getTurbulence();
            
            % Add actuator model
            A = [A zeros(4,1) B; 
                 zeros(2,4), act.A];
            B = [zeros(4,1); act.B];
            C = [C zeros(4,2);
                 zeros(1,4) 0 1];
            %D = zeros(5,1);
            
            % Add turbulence model
            A = [A [zeros(2); this.Qw*turb.C; zeros(2)];
                 zeros(size(turb.A,1),size(A,2)) turb.A];
            B = [[B; zeros(2,1)] [zeros(2,1); this.Qw*turb.D; zeros(2,1); turb.B]];
            C = blkdiag(C, turb.C);
            D = [zeros(5,2); [zeros(2,1) turb.D]];
                 
            % Add accelerometers outputs
            C = [C; A(3:4,:)];
            D = [D; B(3:4,:)];
            % ss([-2*zeta*omega -omega^2; 1 0],[k*omega^2; 0], [0 1], 0);
            
            model = ss(A, B, C, D);
            model.StateName = ['h [ft]           ';
                               'theta [rad]      ';
                               'h_dot [ft/s]     ';
                               'theta_dot [rad/s]';
                               'delta_dot [rad/s]';
                               'delta [rad]      ';
                               'turb state 1     ';
                               'turb state 2     '];
            
            model.InputName = ['delta_c[rad]';
                               'turb in     '];
            
            model.OutputName = ['h [ft]           ';
                                'theta [rad]      ';
                                'h_dot [ft/s]     ';
                                'theta_dot [rad/s]';
                                'delta [rad]      ';
                                'w [m/s]          ';
                                'w dot [m/s]      ';
                                'h_dd [m/s2]      ';
                                'theta_dd [rad/s2 '];
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
        
        % Analitycal model with turbulence
        function model = getModelMatTurb(this)
            delta = @this.inputSignal;
            turb = this.params.getTurbulence(this.U0);
            function ydot = Model(t, y)
                ydot = zeros(6,1);
                ydot(1:2) = y(3:4); % hdot thetadot 
                
                ydot(3:4) = (this.M)\(this.Mg*this.g*cos(this.params.alpha0)...
                    + this.QT*this.params.alpha0 + this.B0*delta(t)...
                    - this.K*y(1:2) - this.D*y(3:4));
                ydot(5:6) = turb.A * y(5:6);
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
        
        function [alt, Uf, qf] = flutterSpeedVersusAlt(this, alt)
            if ~exist('alt','var')
                alt = 0:1000:11000;
            end
            Uf = zeros(1, length(alt));
            qf = zeros(1, length(alt));
            lastU = 30;
            for i = 1:length(alt)
                this.atmosphere.h = alt(i);
                Uf(i) = this.getFlutterSpeed(lastU);
                qf(i) = 0.5*this.atmosphere.rho*Uf(i)^2;
                lastU = Uf(i);
            end
            if nargout == 0
                plot(Uf*3.6, alt, this.lineFormat);
                ylabel('Altitude [m]');
                xlabel('Flutter speed [km/h]')
                title('Flutter speed versus altitude');
            end
        end
        
        function [Gnom Garray] = getPlantArray(this, fuelRange, altRange, speedRange)
            plane = PlaneParams(this.params);
            if ~exist('fuelRange','var')
                fuelRange = plane.fuelLevel;
            end
            if ~exist('altRange','var')
                altRange = this.atmosphere.h;
            end
            if ~exist('speedRange','var')
                speedRange = this.U0;
            end
            if length(fuelRange) == 1 && length(altRange) == 1 && length(speedRange) == 1
                error('No range defined, doesnt make any sense');
            end
            
            % Signals used in feedback control and controller evaluation
            inSignal = [1 2];           % delta_c turb
            outSignal = [1 2 8 9];      % h theta h_dd theta_dd
            
            %inSignal = [1];           % delta_c turb
            %outSignal = [9];      % h theta h_dd theta_dd
            
            % Nominal model:
            Gnom = this.getLinearModel;
            Gnom = Gnom(outSignal,inSignal);
            
            % Create a model family Gr (real)
            %Garray = stack(1,Gnom);
            
            % TODO Make a deep copy of this, don't want to change params
            wing = this;

            for H = altRange
                wing.atmosphere.h = H;
                for fuel = fuelRange
                    plane.fuelLevel = fuel;
                    for speed = speedRange
                        wing.U0 = speed;
                        sys = wing.getLinearModel;
                        sys = sys(outSignal,inSignal);
                        if ~exist('Garray','var')
                            Garray = stack(1,sys);
                        else
                            Garray = stack(1,Garray,sys);
                        end
                    end
                end
            end
        end
        
        function [legnd] = plotPlantArray(this, Gnom, Garray, plot)
            % plot = 1 - standard plot
            % plot = 2 - relative error plot
            % plot = 3 - absolute error plot
            assert(nargin == 4);
            legnd = {};
            
            % Prepare semilogx figure (workaround for Matlab bug)
            figure;
            h = semilogx(1,0);
            hasbehavior(h,'legend',false);
            hold on; % we can do hold on after first plot - otherwise it will be normal not log

            [svGnom wGnom] = sigma(Gnom);
            
            % Plot all real plants
            for i = 1:size(Garray,3)
                [sv w] = sigma(Garray(:,:,i), wGnom);
                if plot == 1
                    h = semilogx(w, mag2db(sv(1,:)), 'k:');
                elseif plot == 2
                    tmp = (sv(1,:) - svGnom(1,:))./svGnom(1,:);
                    %if min(abs(tmp)) < 0.0001
                        %h = semilogx(w, mag2db(tmp), 'k:');
                        h = semilogx(w, abs(tmp), 'k:');
                    %else
                    %    fprintf('Gnom Skipped - min(sigma) = %f\n', min(tmp));
                    %end
                else
                    tmp = sv(1,:) - svGnom(1,:);
                    %if min(abs(tmp)) < 0.0001
                        %h = semilogx(w, mag2db(tmp), 'k:');
                        h = semilogx(w, abs(tmp), 'k:');
                    %else
                    %    fprintf('Gnom Skipped\n');
                    %end
                end
                hasbehavior(h,'legend',false);
            end
            % Ok we want to have one plot that could have legend attached
            hasbehavior(h,'legend',true);
            if plot == 1
                legnd{end+1} = 'G_{ri}';
            elseif plot == 2
                legnd{end+1} = '(G_{ri}(s) - G_n(s))G_n(s)^{-1}';
            else
                legnd{end+1} = 'G_{ri}(s) - G_n(s)';
            end

            % Plot nominal plant
            if plot == 1
                semilogx(wGnom, mag2db(svGnom(1,:)), 'LineWidth', 2, 'Color','r');
                legnd{end+1} = 'G_{n}';
            end
        end
        
        function [W1 W2 Gunc] = addUncert(this, order, varargin)
            % [W1 W2 Gunc] = addUncert(this, order, fuelRange, altRange, speedRange)
            % Calculate weighting function w(s) of multiplicative
            % uncertainty at model output
            % Wo - weighting function of defined order
            % Woi - cell of weigthing functions of other orders < order
            % Gunc - uncertain model generated by ucover function
            
            if ~exist('order','var')
                order = 3;
            end
            
            %plot = 1;      % ordinary plot
            plot = 3;       % absolute error plot
            
            [Gnom Garray] = this.getPlantArray(varargin{:});
            
            legnd = this.plotPlantArray(Gnom, Garray, plot);
            
            % Calculate the weighting function of requested order
              % Output Multiplicative:    USYS = (I + W1*ULTIDYN*W2)*PNOM
              % ord W1 = order
              % ord W2 = []
            [svGnom w] = sigma(Gnom);
            sys = tf(1,1); % - used to transform const weights to system
            
            [Gunc,Info] = ucover(Garray,Gnom,order,[],'Additive');
            W1 = Info.W1 * sys;
            W2 = Info.W2 * sys;
            svW1 = sigma(W1, w);
            svW2 = sigma(W2, w);
            if plot == 1
                semilogx(w, mag2db(svGnom(1,:) + svW1(1,:) .* svW2(1,:)), 'b--');
                legnd{end+1} = 'G_n(s) + W_1(s)\Delta(s)';
            else
                %semilogx(w, mag2db(svW1(1,:) .* svW2(1,:)), 'b--');
                semilogx(w, svW1(1,:) .* svW2(1,:), 'b--');
                legnd{end+1} = 'W_2(s)';
            end
            
            [Gunc,Info] = ucover(Garray,Gnom,[],order,'Additive');
            W1 = Info.W1 * sys;
            W2 = Info.W2 * sys;
            svW1 = sigma(W1, w);
            svW2 = sigma(W2, w);
            if plot == 1
                semilogx(w, mag2db(svGnom(1,:) + svW1(1,:) .* svW2(1,:)), 'c--');
                legnd{end+1} = 'G_n(s) + \Delta(s)W_2(s)';
            else
                %semilogx(w, mag2db(svW1(1,:) .* svW2(1,:)), 'c--');
                semilogx(w, (svW1(1,:) .* svW2(1,:)), 'c--');
                legnd{end+1} = 'W_2(s)';
            end
            
            [Gunc,Info] = ucover(Garray,Gnom,order,order,'Additive');
            W1 = Info.W1 * sys;
            W2 = Info.W2 * sys;
            svW1 = sigma(W1, w);
            svW2 = sigma(W2, w);
            if plot == 1
                semilogx(w, mag2db(svGnom(1,:) + svW1(1,:) .* svW2(1,:)), 'g--');
                legnd{end+1} = 'G_n(s) + W_1(s)\Delta(s)W_2(s)';
            else
                %semilogx(w, mag2db(svW1(1,:) .* svW2(1,:)), 'g--');
                semilogx(w, (svW1(1,:) .* svW2(1,:)), 'g--');
                legnd{end+1} = 'W_1(s)W_2(s)';
            end
            
            %legend('G_{ri}' ,'G_n', 'G_n + W_1 \Delta','G_n + \Delta W2','G_n + W_1 \Delta W_2')
            legend(legnd);
        end
        
        function [W1 W2 Gunc] = multUncert(this, order, varargin)
            % [W1 W2 Gunc] = multUncert(this, order, fuelRange, altRange, speedRange)
            % Calculate weighting function w(s) of multiplicative
            % uncertainty at model output
            % W1 W2 - weighting function of defined order
            % Gunc - uncertain model generated by ucover function
            
            if ~exist('order','var')
                order = 3;
            end
            
            %plot = 1;  % standard plot
            plot = 2;  % error plot
            
            [Gnom Garray] = this.getPlantArray(varargin{:});

            legnd = this.plotPlantArray(Gnom, Garray, plot);
            %this.plotPlantArray(Gnom, Garray, plot);
            
            % Calculate the weighting function of requested order
              % Output Multiplicative:    USYS = (I + W1*ULTIDYN*W2)*PNOM
              % ord W1 = order
              % ord W2 = []
            uncertType = 'OutputMult';            
            [svGnom w] = sigma(Gnom);
            sys = tf(1,1); % - used to transform const weights to system
            
            [Gunc,Info] = ucover(Garray,Gnom,order,[],uncertType);
            W1out = Info.W1 * sys;
            W2out = Info.W2 * sys;
            svW1 = sigma(W1out, w);
            svW2 = sigma(W2out, w);
            if plot == 1
                semilogx(w, mag2db((1 + svW1(1,:) .* svW2(1,:)).*svGnom(1,:)), 'b');
                legnd{end+1} = 'Out 1';
            else
                %semilogx(w, mag2db(svW1(1,:) .* svW2(1,:)), 'b');
                semilogx(w, (svW1(1,:) .* svW2(1,:)), 'b');
                legnd{end+1} = 'Out W_1(s)';
            end

            [Gunc,Info] = ucover(Garray,Gnom,order,order,uncertType);
            W1out = Info.W1 * sys;
            W2out = Info.W2 * sys;
            svW1o = sigma(W1out, w);
            svW2o = sigma(W2out, w);
            if plot == 1
                semilogx(w, mag2db((1 + svW1o(1,:) .* svW2o(1,:)).*svGnom(1,:)), 'b:');
                legnd{end+1} = 'Out 2';
            else
                %semilogx(w, mag2db(svW1o(1,:) .* svW2o(1,:)), 'b:');
                semilogx(w, (svW1o(1,:) .* svW2o(1,:)), 'b:');
                legnd{end+1} = 'Out W_1(s)W_2(s)';
            end
            %semilogx(w, mag2db(svGnom(1,:).*(1 + svW1(1,:))), 'm:');
            %semilogx(w, mag2db(svGnom(1,:).*(1 + svW2(1,:))), 'y-.');
            
            uncertType = 'InputMult';
            [Gunc,Info] = ucover(Garray,Gnom,order,[],uncertType);
            W1in = Info.W1 * sys;
            W2in = Info.W2 * sys;
            svW1i = sigma(W1in, w);
            svW2i = sigma(W2in, w);
            if plot == 1
                semilogx(w, mag2db(svGnom(1,:).*(1 + svW1i(1,:) .* svW2i(1,:))), 'g');
            else
                %semilogx(w, mag2db(svW1i(1,:) .* svW2i(1,:)), 'g');
                semilogx(w, (svW1i(1,:) .* svW2i(1,:)), 'g');
            end
            legnd{end+1} = 'In 1';
            
            [Gunc,Info] = ucover(Garray,Gnom,order,order,uncertType);
            W1in = Info.W1 * sys;
            W2in = Info.W2 * sys;
            svW1i = sigma(W1in, w);
            svW2i = sigma(W2in, w);
            if plot == 1
                semilogx(w, mag2db(svGnom(1,:).*(1 + svW1i(1,:) .* svW2i(1,:))), 'g:');
            else
                %semilogx(w, mag2db(svW1i(1,:) .* svW2i(1,:)), 'g:');
                semilogx(w, (svW1i(1,:) .* svW2i(1,:)), 'g:');
            end            
            legnd{end+1} = 'In 2';
            
            % Input - output 
%             if plot == 1
%                 semilogx(w, mag2db((1 + svW1o(1,:) .* svW2o(1,:)).*svGnom(1,:).*(1 + svW1i(1,:) .* svW2i(1,:))), 'g:');
%                 legnd{end+1} = 'In - Out';
%             else
%                 %semilogx(w, mag2db(svW1i(1,:) .* svW2i(1,:)), 'b');
%             end
            
            %legend('G_{ri}' ,'G_n', 'G_n(1 + W_1 \Delta)','G_n(1 + W_1 \Delta W_2)')
            legend(legnd);

            W1 = W1in;
            W2 = W2in;
        end
        
        function val = get.actuator(this)
            k = 1.02;           % [dg/deg]
            zeta = 0.56;        % [-]
            omega = 165.3;      % [rad/s]
            %val = tf(k*omega^2, [1 2*zeta*omega omega^2]);
            val = ss([-2*zeta*omega -omega^2; 1 0],[k*omega^2; 0], [0 1], 0);
            
%             if strcmp(this.actuatorModel, 'off')
%                 val = tf(1, 1);
%             end
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
                s = value;
            end
            fun = @signal;
        end
    end
    
end

