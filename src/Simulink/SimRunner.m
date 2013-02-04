classdef SimRunner < handle
    properties
        mdl             = 'WingSim'
        wingFlutter
        thisName
        % Default input signal
        t = [0 1 1 2]'
        u = [0 0 1 1]'
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
            this.t = (0:0.01:stopTime)';
            this.u = arrayfun(this.wingFlutter.stepSignal,this.t);
        end
        function [t y] = sim(this, stopTime)
            % Prepare input signal
            this.getInputSignal(stopTime);
            % Load model
            load_system(this.mdl);
            % Prepare model configuration
            conf = getActiveConfigSet(this.mdl);
            cs = conf.copy();
            set_param(cs, 'StopTime', num2str(stopTime));
            set_param(cs, 'LoadExternalInput', 'on');
            set_param(cs, 'ExternalInput', [ '[' this.thisName '.t,' this.thisName '.u]' ]);  % <-- 1
            % Run simulation
            simout = sim(this.mdl, cs);
            % Plot results
            t = simout.find('tout');
            y = simout.find('yout');
            
            if nargout == 0
                subplot(2,1,1); hold on;
                plot(t, y(:,1), 'b--');
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
