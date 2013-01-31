classdef Bact < handle
    %BACT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(GetAccess = 'public', SetAccess = 'public')
        Temop0 = 288.15;    % Sea level standard temperature [K]
        P0 = 101325;        % Sea level standard atmospheric pressure [Pa]
        U0 = 100;
        q = 100;
        thetaT = 0
                % speed of sound
        Cs = 1115.49;       % ft/s (for T=20 C - I guess that's the temp. inside test tunnel))
                            % 340 m/s
        isGravity = 'on'
        ActuatorOutputs = 'on'
        inputSignal = Bact.stepSignal()
        lineFormat = 'b'
    end
    
    properties(Constant)

        g = 32.2;           % ft/s2
        % rho = 0;            % this was variable in test tunnel
        
        % Stale strukturalne
        m =             6.08;           % slug
        Itheta =        2.8;            % slug*ft2
        Kh =            2686;           % lb/ft
        Ktheta =        3000;           % ft-lb
        omegah =        21.01;          % rad/s
        omegatheta =    32.72;          % rad/s
        zetah =         0.0014;         % -
        zetatheta =     0.001;          % -
        shtheta =       0.0142;         % slug-ft
        shdelta =       0.00288;        % slug-ft
        sthetadelta =   0.00157;        % slug-ft2
        
        % Stale aerodynamiczne
        CLsign = -1;    % konwencja znaku jest porypana
        %CLalpha =       4.584;          % 1/rad
        CLalpha =      -4.584;          % 1/rad
        CMalpha =       1.490;          % 1/rad
        %CLalphadot =   -3.1064;         % 1/rad
        CLalphadot =    3.1064;         % 1/rad
        %CLq =           2.5625;         % 1/rad
        CLq =          -2.5625;         % 1/rad
        CMalphadot =   -2.6505;         % 1/rad
        CMq =          -0.4035;         % 1/rad
        %CLdelta =       0.63;           % 1/rad
        CLdelta =      -0.63;           % 1/rad
        CLdeltadot =    0;              % 1/rad
        CMdelta =      -0.0246;         % 1/rad
        CMdeltadot =    0;              % 1/rad
        S =             3.55;           % ft2
        c =             1.33;           % ft
        % l = -0.175 %c
        l =            -0.175 * 1.33;      % ft
        
        % Stale przyspieszeniomierzy (actuators data)
        % Tylko te polozone na koncu skrzydla
        Dleo =         -0.599           % ft
        Dteo =          0.420           % ft
    end
    
    properties(GetAccess = 'public', SetAccess = 'private')
        % Macierzowa postac parametrow strukturalnych
        Ms
        Ks
        Ds
        
        % Opis sil aero
        Ma
        Ka
        Da
        QT
        % Sterowanie
        B0
        B1
        B2
        % Wypadkowe macierze
        M
        D
        K
    end
    
    properties(Dependent)
        % Wplyw grawitacji
        Mg
    end
    
    methods
        function this = Bact(U0, q)
             % this.Cs = (331.3 + 0.606*this.Temp) * 3.28084; % ft/s
             this.U0 = U0;
             this.q = q;
        end
        
        function init(this)
            % Macierzowa postac parametrow strukturalnych
            this.Ms = [this.m this.shtheta; this.shtheta this.Itheta];
            this.Ks = [this.Kh 0; 0 this.Ktheta];
            this.Ds = this.Ms*[2*this.zetah*this.omegah 0; 0 2*this.zetatheta*this.omegatheta]; %TODO
            %this.Mg = [this.m; this.shtheta];
            
            % Aerodynamic model
            this.Ma = this.q*this.S*this.c/(2*this.U0^2) * ...
                [this.CLalphadot, this.l*this.CLalphadot; 
                this.c*this.CMalphadot, this.c*this.l*this.CMalphadot];
            
            this.Da = this.q*this.S/this.U0 *... 
                [this.CLalpha ,  this.l*this.CLalpha  + this.c/2 * (this.CLalphadot+this.CLq);
                 this.c*this.CMalpha,  this.c*this.l*this.CMalpha + this.c*this.c/2*(this.CMalphadot+this.CMq)];
             
            this.Ka = this.q*this.S*[0 this.CLalpha; 0  this.c*this.CMalpha];
            this.QT = this.q*this.S*[this.CLalpha; this.c*this.CMalpha];
            this.B0 = this.q*this.S*[this.CLdelta; this.c*this.CMdelta];
            
            % Full matrices
            this.M = this.Ms - this.Ma;
            this.D = this.Ds - this.Da;
            this.K = this.Ks - this.Ka;
        end
        
        function set.isGravity(this, is)
            assert(strcmp(is,'on') || strcmp(is,'off'))
            this.isGravity = is;
        end
        
        function Mg = get.Mg(this)
            if strcmp(this.isGravity, 'on')
                Mg = [this.m; this.shtheta];
            else
                Mg = [0; 0];
            end
        end
        
        function set.U0(this,U0)
            this.U0 = U0;
            this.init();
        end
        
        function set.q(this,q)
            this.q = q;
            this.init();
        end
        
        function set.thetaT(this,thetaT)
            this.thetaT = thetaT;
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
                    0 0 -1/this.g -this.Dleo/this.g; 
                    0 0 -1/this.g  this.Dteo/this.g];
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
                
                ydot(3:4) = (this.M)\(this.Mg*this.g*cos(this.thetaT)...
                    + this.QT*this.thetaT + this.B0*delta(t)...
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
                ydot(3:4) = (this.Ms)\(this.Mg*this.g*cos(this.thetaT)...
                    - this.Ks*y(1:2) - this.Ds*y(3:4));
                % -------------- sily aero
                h = y(1);
                theta = y(2);
                hdot = y(3);
                thetadot = y(4);

                alpha = this.thetaT + theta + hdot/this.U0 + this.l*thetadot/this.U0;
                alphadot = thetadot; % 
                CL = this.CLalpha*alpha + this.CLdelta*delta...
                    + this.c/(2*this.U0) * (this.CLalphadot*alphadot + this.CLq * thetadot);
                Lift = this.q*this.S*CL;
                CM = this.CMalpha*alpha + this.CMdelta*delta...
                    + this.c/(2*this.U0) * (this.CMalphadot*alphadot + this.CMq*thetadot);
                Torque = this.q*this.S*this.c*CM;

                ydot(3) = ydot(3) + Lift/this.m;
                ydot(4) = ydot(4) + Torque/this.Itheta;
            end
            model = @BactModel;
        end
        
        function state = trim(this, thetaT, delta)
            this.thetaT = thetaT;
            % TODO co to jest Q0e ?
%             state = this.K\(this.Q0e + this.QT*thetaT...
%                 + this.Mg*this.g*cos(thetaT) + this.B0*delta);
            state = this.K\(           this.QT*thetaT...
                + this.Mg*this.g*cos(thetaT) + this.B0*delta);
            
            % state = this.K\(this.QT*thetaT + this.Mg*this.g*cos(thetaT));
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
            subplot(2,1,1); 
            ylabel('h [ft]');
            plot(t, y(:,1))
            subplot(2,1,2); 
            ylabel('theta [deg]');
            plot(t, y(:,2)*180/pi)
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
            this.M = this.Ms - this.Ma;
            model2 = this.getModelMat();
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
            fprintf('\tturntable theta: \t%f\n', this.thetaT);
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

