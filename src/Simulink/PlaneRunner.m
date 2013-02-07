classdef PlaneRunner < SimRunner
    properties
        stabParams
        fuse
    end
    
    methods
        function this = PlaneRunner(thisName, wingFlutter, mdl)   % Constructor
            this = this@SimRunner(thisName);
                
            if nargin == 3
                this.mdl = mdl;
            else
                this.mdl = 'PlaneSim';
            end
            
            if nargin >= 2
                this.wingFlutter = wingFlutter;
            else
                wingParams = WingParams();
                this.wingFlutter = WingFlutter(wingParams);
            end
            
            this.thisName = thisName;
            this.stabParams = StabParams;
            this.initFuselageParams();
        end
        function getInputSignal(this, stopTime)
            % TODO Make t, u dependent variables
            this.t = (0:0.01:stopTime)';
            %this.u = arrayfun(this.wingFlutter.inputSignal,this.t);
            this.u = zeros(size(this.t,1), 4);
        end
        function setState(this, state)
            this.state = [0 0 0 0 0 0];
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
            set_param(cs, 'SaveOutput', 'on');
            set_param(cs, 'OutputSaveName', 'yout');
            set_param(cs, 'SaveTime', 'on');
            set_param(cs, 'OutputTimes', 'tout');
            %set_param(cs, 'LoadInitialState', 'on');
            %set_param(cs, 'InitialState', [ this.thisName '.state' ]);
            
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
        
        function initFuselageParams(this)
            this.fuse.K.K_theta = 1e6;
            this.fuse.K.K_psi = 1e6;
            this.fuse.K.K_phi = 1e4;
            this.fuse.K.Zeta_theta = 0.001;
            this.fuse.K.Zeta_psi = 0;
            this.fuse.K.Zeta_phi = 0.0014;
            
            this.fuse.Central.Mass = 20; % kg
            this.fuse.Central.Inertia = [3.2 0 0; 0 3.2 0; 0 0 3.2]; % kg*m2

            this.fuse.Back.Mass = 60 + 75; % kg
            this.fuse.Back.Inertia = [1138 0 0; 0 1138 0; 0 0 14.4]; % kg*m2
            this.fuse.Back.CG = [0 0 2.52];

            this.fuse.Front.Mass = 60 + 140; % kg
            this.fuse.Front.Inertia = [785 0 0; 0 785 0; 0 0 10.8]; % kg*m2
            this.fuse.Front.CG = [0 0 -2.68];
            
            assert(det(this.fuse.Central.Inertia) ~= 0);
            assert(det(this.fuse.Front.Inertia) ~= 0);
            assert(det(this.fuse.Back.Inertia) ~= 0);
        end
    end
end
