classdef SimRunner < handle
    properties
        mdl             = 'WingSim'
        wingFlutter
        thisName
        % Default input signal
        t = [0 1 1 2]'
        u = [0 0 1 1]'
        state
    end
    
    properties(Dependent)
        U0, Q, g
        aeroForceOn
        params
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
        function getInputSignal(this, stopTime)
            % TODO Make t, u dependent variables
            this.t = (0:0.05:stopTime)';
            this.u = arrayfun(this.wingFlutter.stepSignal,this.t);
        end
        function setState(this, state)
            this.state = [0 0 0 0 0 0];
            % internal internal theta psi theta_dot psi_dot
%             this.state(5) = state(2);                  % theta
%             this.state(6) = state(1)*this.params.Yac;  % convert h to phi
%             this.state(3) = state(4);                  % theta_dot
%             this.state(4) = state(3)*this.params.Yac;  % convert h_dot to phi_dot
            this.state(1) = state(2);                  % theta
            this.state(2) = state(1)*this.params.Yac;  % convert h to phi
            this.state(3) = state(4);                  % theta_dot
            this.state(4) = state(3)*this.params.Yac;  % convert h_dot to phi_dot
        end
        function [t y] = sim(this, stopTime, state)
            % Prepare input signal
            if nargin == 1
                stopTime = 10;
            end
            this.getInputSignal(stopTime);
            % Prepare initial state
            if nargin > 2
                this.setState(state);
            else
                this.state = [0 0 0 0 0 0];
            end
            % Load model
            load_system(this.mdl);
            % Prepare model configuration
            conf = getActiveConfigSet(this.mdl);
            cs = conf.copy();
            set_param(cs, 'StopTime', num2str(stopTime));
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
        function val = get.params(this)
            val = this.wingFlutter.params;
        end
    end
end
