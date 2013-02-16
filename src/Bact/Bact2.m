classdef Bact2 < handle
    %BACT Summary of this class goes here
    %   Detailed explanation goes here
    properties(Constant)
        g = 32.2;           % ft/s2
        Temop0 = 288.15;    % Sea level standard temperature [K]
        P0 = 101325;        % Sea level standard atmospheric pressure [Pa]
    end
    properties(GetAccess = 'public', SetAccess = 'public')
        U0 = 400;
        q = 80;
        % thetaT = 0
                % speed of sound
        Cs = 1115.49;       % ft/s (for T=20 C - I guess that's the temp. inside test tunnel))
                            % 340 m/s
        isGravity = 'on'
        ActuatorOutputs = 'off'
        inputSignal = Bact2.stepSignal()
        lineFormat = 'b'
        params;
        simStyle = 'k'
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
    end
    
    methods
        function this = Bact2(params)
            this.params = params;
        end
        
        function Ms = get.Ms(this)
            Ms = [this.params.m this.params.shtheta; this.params.shtheta this.params.Itheta];
        end
        
        function Ks = get.Ks(this)
            Ks = [this.params.Kh 0; 0 this.params.Ktheta];
        end
        
        function Ds = get.Ds(this)
            Ds = this.Ms*[2*this.params.zetah*this.params.omegah 0; 0 2*this.params.zetatheta*this.params.omegatheta]; %TODO
        end
        
        function Ma = get.Ma(this)
            Ma = this.q*this.params.S*this.params.c/(2*this.U0^2) * ...
                [this.params.CLalphadot, this.params.l*this.params.CLalphadot; 
                this.params.c*this.params.CMalphadot, this.params.c*this.params.l*this.params.CMalphadot];
        end
        
        function Da = get.Da(this)
            Da = this.q*this.params.S/this.U0 *... 
                [this.params.CLalpha ,  this.params.l*this.params.CLalpha  + this.params.c/2 * (this.params.CLalphadot+this.params.CLq);
                 this.params.c*this.params.CMalpha,  this.params.c*this.params.l*this.params.CMalpha + this.params.c*this.params.c/2*(this.params.CMalphadot+this.params.CMq)];
        end
        
        function Ka = get.Ka(this)
            Ka = this.q*this.params.S*[0 this.params.CLalpha; 0  this.params.c*this.params.CMalpha];
        end
        
        function QT = get.QT(this)
            QT = this.q*this.params.S*[this.params.CLalpha; this.params.c*this.params.CMalpha];
        end
        
        function B0 = get.B0(this)
            B0 = this.q*this.params.S*[this.params.CLdelta; this.params.c*this.params.CMdelta];
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
                Mg = [this.params.m; this.params.shtheta];
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
            
            if strcmp(this.ActuatorOutputs, 'on');
                C = [C; 
                    0 0 -1/this.g -this.params.Dleo/this.g; 
                    0 0 -1/this.g  this.params.Dteo/this.g];
                % TODO tutaj w D brakuje wplywu deltadotdot na przysp.
                D = zeros(6,1);
            end
            
            model = ss(A, B, C, D);
            model.InputName = 'delta[rad]';
            % TODO latex symbols
            model.StateName = ['h [ft]           ';
                               'theta [rad]      ';
                               'h_dot [ft/s]     ';
                               'theta_dot [rad/s]'];
            if strcmp(this.ActuatorOutputs, 'on');
                model.OutputName = [model.StateName;
                               'acc leading [g]  ';
                               'acc trailing [g] '];
            else
                model.OutputName = model.StateName;
            end
        end
        
        function model = getModelMat(this)
            delta = @this.inputSignal;
            function ydot = BactModel(t, y)
                ydot = zeros(4,1);
                ydot(1:2) = y(3:4); % hdot thetadot 
                
                ydot(3:4) = (this.M)\(this.Mg*this.g*cos(this.params.alpha0)...
                    + this.QT*this.params.alpha0 + this.B0*delta(t)...
                    - this.K*y(1:2) - this.D*y(3:4));
            end
            model = @BactModel;
        end
        
        function model = getModelSim(this)
            delta = 0;
            function ydot = BactModel(t, y)
                ydot = zeros(4,1);
                ydot(1:2) = y(3:4); % hdot thetadot 
                % -------------- sily strukturalne
                ydot(3:4) = (this.Ms)\(this.Mg*this.g*cos(this.params.alpha0)...
                    - this.Ks*y(1:2) - this.Ds*y(3:4));
                % -------------- sily aero
                h = y(1);
                theta = y(2);
                hdot = y(3);
                thetadot = y(4);

                alpha = this.params.alpha0 + theta + hdot/this.U0 + this.params.l*thetadot/this.U0;
                alphadot = thetadot; % 
                CL = this.params.CLalpha*alpha + this.params.CLdelta*delta...
                    + this.params.c/(2*this.U0) * (this.params.CLalphadot*alphadot + this.params.CLq * thetadot);
                Lift = this.q*this.params.S*CL;
                CM = this.params.CMalpha*alpha + this.params.CMdelta*delta...
                    + this.params.c/(2*this.U0) * (this.params.CMalphadot*alphadot + this.params.CMq*thetadot);
                Torque = this.q*this.params.S*this.params.c*CM;

                ydot(3) = ydot(3) + Lift/this.params.m;
                ydot(4) = ydot(4) + Torque/this.params.Itheta;
            end
            model = @BactModel;
        end
        
        function state = trim(this, thetaT, delta)
            this.params.alpha0 = thetaT;
%             state = this.K\(this.Q0e + this.QT*thetaT...
%                 + this.Mg*this.g*cos(thetaT) + this.B0*delta);
            state = this.K\(           this.QT*thetaT...
                + this.Mg*this.g*cos(thetaT) + this.B0*delta);
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
        
        function sim(this, stopTime, state)
            if ~exist('stopTime','var')
                stopTime = 10;
            end
            if ~exist('state','var')
                state = [0 0 0 0];
            end
            [t y] = ode45(this.getModelMat(), [0 stopTime], state);
            subplot(2,1,1); hold on;
            ylabel('h [ft]');
            plot(t, y(:,1), this.simStyle)
            subplot(2,1,2); hold on; 
            ylabel('theta [deg]');
            plot(t, y(:,2)*180/pi, this.simStyle)
        end
        
        function setInputSignal(this, type)
            type = lower(type);
            if strcmp(type, 'const')
                this.inputSignal = this.constSignal();
            elseif strcmp(type, 'step')
                this.inputSignal = this.stepSignal();
            end
        end
        
        function compareSimMat(this, stopTime)
            if ~exist('stopTime','var')
                stopTime = 4;
            end
            this.compareModels({this.getModelMat(), this.getModelSim()}, stopTime);
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
        
        function Uf = SpeedRootLocus(this, U, rho, format)
            if ~exist('U', 'var')
                U = 100:50:400;
            end
            if ~exist('format', 'var')
                format = 'b.';
            end
            Uf = Inf;
%             if ~exist('Q', 'var')
%                 Q = 80:10:140;
%             end
%             rho = 0.00217; % slug/ft3
            locus = zeros(length(U), 4);
            for i = 1:length(U)
                this.U0 = U(i);
                this.q = (rho*this.U0^2)/2;
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
                fprintf('Flutter for U = %f, Q = %f psf\n', Uf, Uf^2*rho/2);
            end
        end
        
        function disp(this)
            fprintf('BACT model for:\n')
            fprintf('\tspeed: \t\t%f [ft/s]\n', this.U0);
            fprintf('\tMach: \t\t%f\n', this.U0/this.Cs);
            fprintf('\tdyn. pressure: \t%f [psf]\n', this.q);
            fprintf('\tturntable theta: \t%f\n', this.params.alpha0);
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

