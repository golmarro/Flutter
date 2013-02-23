classdef SimRunner < handle
    properties
        mdl             = 'WingSim'
        wingFlutter
        thisName
        % Default input signal
        stopTime = 10
        state = []
        actuatorModel = 'linear'
    end
    
    properties(Dependent)
        U0, Q, g, rho
        aeroForceOn
        wingParams
        actuator
        turbulence
        t, u         % input signal
    end
    
    methods
        function this = SimRunner(thisName, wingFlutter, mdl)   % Constructor
            if nargin == 3
                this.mdl = mdl;
            elseif nargin == 2
                this.wingFlutter = wingFlutter;
            else
                wingParams = WingParams();
                this.wingFlutter = WingFlutter(wingParams);
            end
            
            this.thisName = thisName;
        end
        function val = get.u(this)
            val = [arrayfun(this.wingFlutter.inputSignal,this.t),...
                   arrayfun(this.wingFlutter.turbulenceInputSignal, this.t)];
        end
        function val = get.t(this)
            val = (0:0.01:this.stopTime)';
        end
        function setState(this, state)
            this.state = [0 0 0 0 0 0 0 0];
            % internal internal theta psi theta_dot psi_dot
%             this.state(5) = state(2);                  % theta
%             this.state(6) = state(1)*this.params.Yac;  % convert h to phi
%             this.state(3) = state(4);                  % theta_dot
%             this.state(4) = state(3)*this.params.Yac;  % convert h_dot to phi_dot
            this.state(1) = state(2);                  % theta
            this.state(2) = state(1)*this.wingParams.Yac;  % convert h to phi
            this.state(3) = state(4);                  % theta_dot
            this.state(4) = state(3)*this.wingParams.Yac;  % convert h_dot to phi_dot
        end
        function [t y] = sim(this, stopTime, state)
            % Prepare input signal
            if nargin >= 2
                this.stopTime = stopTime;
            end
            %this.getInputSignal(stopTime);
            % Prepare initial state
            if nargin > 2
                this.setState(state);
            else
                this.state = [0 0 0 0 0 0 0 0];
            end
            % Load model
            load_system(this.mdl);
            % Prepare model configuration
            conf = getActiveConfigSet(this.mdl);
            cs = conf.copy();
            set_param(cs, 'StopTime', num2str(this.stopTime));
            set_param(cs, 'RelTol', '1e-6');
            set_param(cs, 'LoadExternalInput', 'on');
            set_param(cs, 'ExternalInput', [ '[' this.thisName '.t,' this.thisName '.u]' ]);  % <-- 1
            set_param(cs, 'LoadInitialState', 'on');
            set_param(cs, 'InitialState', [ this.thisName '.state' ]);
            % Run simulation
            simout = sim(this.mdl, cs);
            % Plot results
            t = simout.find('tout');
            y = simout.find('yout');
            
            if nargout == 0
                subplot(2,1,1); hold on;
                plot(t, -y(:,1), 'b--');
                ylabel('h [m]')
                subplot(2,1,2); hold on;
                plot(t, 180/pi*y(:,2), 'b--');
                xlabel('time [s]'); ylabel('\theta [deg]');
            end
        end
        
        function val = get.U0(this)
            val = this.wingFlutter.U0;
        end
        function val = get.Q(this)
            val = this.wingFlutter.q;
        end
        function val = get.rho(this)
            val = this.wingFlutter.atmosphere.rho;
        end
        function val = get.g(this)
            if strcmp(this.wingFlutter.isGravity,'off')
                val = 0;
            else
                val = this.wingFlutter.g;
            end
        end
        function val = get.aeroForceOn(this)
            if strcmp(this.wingFlutter.AeroForces,'off')
                val = 0;
            else
                val = 1;
            end
        end
        function val = get.wingParams(this)
            val = this.wingFlutter.params;
        end
        function val = get.actuator(this)
            val.tf = this.wingFlutter.actuator;
            
            if strcmp(this.actuatorModel, 'nonlinear')
                val.rateLimit = Inf;    % TODO ????
                val.rangeLimit = 45 * pi/180;
            elseif strcmp(this.actuatorModel, 'linear')
                val.rateLimit = Inf;
                val.rangeLimit = Inf;
            else        %  actuatorModel = 'none'
                val.rateLimit = Inf;
                val.rangeLimit = Inf;
                val.tf = tf(1, 1);
            end
        end
        function val = get.turbulence(this)
            val = this.wingFlutter.params.getTurbulence();
        end
    end
end
