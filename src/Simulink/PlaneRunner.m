classdef PlaneRunner < SimRunner
    properties
        stabParams
        fuse
        deltaTrim = [0 0 0 0]*pi/180 % delta values after trim
        lastOperPoint = [] % last operationg point evaluated by trim function
        lineStyle = 'k'
        inputSignalType = 'const'
    end
    
    properties(Dependent)
        u_plane       % Can't overwrite get.u getter from SimRunner, so need new var
    end
    
    methods
        function this = PlaneRunner(thisName, mdl, wingFlutter)   % Constructor
            this = this@SimRunner(thisName);
                
            if nargin >= 2
                this.mdl = mdl;
            else
                this.mdl = 'PlaneSim';
            end
            
            if nargin == 3
                this.wingFlutter = wingFlutter;
            else
                wingParams = WingParams();
                this.wingFlutter = WingFlutter(wingParams);
            end
            
            this.thisName = thisName;
            this.stabParams = StabParams;
            this.initFuselageParams();
        end

        function val = get.u_plane(this)
            step = arrayfun(WingFlutter.stepSignal(0.1,5*pi/180),this.t);
            val = repmat(this.deltaTrim, size(this.t,1), 1);
            
            if strcmp(this.inputSignalType, 'const')
                return
            end
            if ~isempty(strfind(this.inputSignalType, 'pitch_up'))
                val(:,3) = val(:,3) - step;
                val(:,4) = val(:,4) - step;
            elseif ~isempty(strfind(this.inputSignalType, 'pitch_down'))
                val(:,3) = val(:,3) + step;
                val(:,4) = val(:,4) + step;
            end
            if ~isempty(strfind(this.inputSignalType, 'roll_right'))
                val(:,1) = val(:,1) - step;
                val(:,2) = val(:,2) + step;
            elseif ~isempty(strfind(this.inputSignalType, 'roll_left'))
                val(:,1) = val(:,1) + step;
                val(:,2) = val(:,2) - step;
            end
            if ~isempty(strfind(this.inputSignalType, 'rudder_right'))
                val(:,3) = val(:,3) - step;
                val(:,4) = val(:,4) + step;
            elseif ~isempty(strfind(this.inputSignalType, 'rudder_left'))
                val(:,3) = val(:,3) + step;
                val(:,4) = val(:,4) - step;
            end
        end
        function setState(this, state)
            this.state = [0 0 0 0 0 0];
        end
        function [t y] = sim(this, stopTime, state)
            % Prepare input signal
            if nargin >= 2
                this.stopTime = stopTime;
            else
                this.stopTime = 10;
            end
            % Prepare initial state
            % TODO setState
%             if nargin > 2
%                 this.setState(state);
%             else
%                 this.state = [0 0 0 0 0 0];
%             end
            % Load model
            load_system(this.mdl);
            % Prepare model configuration
            conf = getActiveConfigSet(this.mdl);
            cs = conf.copy();
            set_param(cs, 'StopTime', num2str(this.stopTime));
            set_param(cs, 'RelTol', '1e-6');
            set_param(cs, 'LoadExternalInput', 'on');
            set_param(cs, 'ExternalInput', [ '[' this.thisName '.t,' this.thisName '.u_plane]' ]);
            set_param(cs, 'SaveOutput', 'on');
            set_param(cs, 'OutputSaveName', 'yout');
            set_param(cs, 'SaveTime', 'on');
            set_param(cs, 'OutputTimes', 'tout');
            if ~isempty(this.state)
                set_param(cs, 'LoadInitialState', 'on');
                set_param(cs, 'InitialState', [ this.thisName '.state' ]);
            end
            
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
        
        function op = trim(this)
            savedActuatorMode = this.actuatorModel;
            %this.actuatorModel = 'none';
            
            DofIndex = 1;
            
            spec = operspec(this.mdl);
            % TODO wspolna identyfikacja stanow dla wszystkich modeli
            % theta, psi, phi,
            
            if strcmp(this.mdl,'PlaneSim')
                offset = 6;
            else
                offset = 0;
            end
            
            if ~strcmp(this.actuatorModel, 'none')
                offset = offset + 4;
            end
            
            set(spec.States(1 + offset),'SteadyState', 1)
            set(spec.States(2 + offset),'Known', 1)
            set(spec.States(3 + offset),'Known', 1)
            set(spec.States(4 + offset),'Known', 1)  % pos
            set(spec.States(5 + offset),'Known', 1)
            set(spec.States(6 + offset),'Known', 1)  
            set(spec.States(7 + offset),'Known', 1)  % p, q, r
            set(spec.States(8 + offset),'Known', 1)
            set(spec.States(9 + offset),'SteadyState', 0)
            set(spec.States(10 + offset),'SteadyState', 0)    % V
            set(spec.States(11 + offset),'Known', 1)
            set(spec.States(12 + offset),'SteadyState', 0)
            
            
%             for i = 1:size(spec.States,1)
%                 % Actuator states don't have to be in equilibrium
%                 if ~isempty(strfind(get(spec.States(i),'Block'), 'Actuator'))
%                     set(spec.States(i),'SteadyState', [0;0])
%                 % Fakejoint represents planes 6 DoF
%                 elseif ~isempty(strfind(get(spec.States(i),'Block'), 'PlaneDOF'))
%                     if DofIndex == 1
%                         set(spec.States(i),'SteadyState', 1)
%                         set(spec.States(i),'x', 5*pi/180)
%                     else %if DofIndex < 7
%                         set(spec.States(i),'Known', 1)
%                     end
%                     DofIndex = DofIndex + 1;
%                 % All Joints can be distorted
%                 elseif ~isempty(strfind(get(spec.States(i),'Block'), 'Joint'))
%                     % Initial guess for theta:
%                     set(spec.States(i),'SteadyState', 0)
%                 % But cannot move too much
%                 elseif ~isempty(strfind(get(spec.States(i),'Block'), 'Velocity'))
%                     set(spec.States(i),'SteadyState', 1)
%                 end
%             end

            % Ailerons:
            set(spec.Inputs(1), 'Known', 1)
            set(spec.Inputs(2), 'Known', 1)
            % Elevetors:
%             set(spec.Inputs(3), 'Min', -45*pi/180);
%             set(spec.Inputs(3), 'Max', 45*pi/180);
%             set(spec.Inputs(4), 'Min', -45*pi/180);
%             set(spec.Inputs(4), 'Max', 45*pi/180);
            set(spec.Inputs(3), 'u', -10*pi/180);
            set(spec.Inputs(4), 'u', -10*pi/180);
            if strcmp(this.mdl,'PlaneSim')
                set(spec.Inputs(5), 'Known', 1)
                set(spec.Inputs(6), 'Known', 1)
            end
            
            spec
            op = findop(this.mdl,spec);
            
            % Extract states from operating point
            for i = 1:size(spec.States,1)
                %this.state(i) = get(op.States(i),'x');
            end
            % Extract inputs from operating point
            for i=1:4
                this.deltaTrim(i) = get(op.Inputs(i),'u');
            end
            
            this.lastOperPoint = op;
            this.actuatorModel = savedActuatorMode;
        end
        
        function sys = linearize(this)
            if isempty(this.lastOperPoint)
                this.trim();
            end
            
            io = getlinio(this.mdl);
            
            sys = linearize(this.mdl, io);
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
            this.fuse.Back.CG = [0 0 2.32];

            this.fuse.Front.Mass = 60 + 80; % kg
            this.fuse.Front.Inertia = [785 0 0; 0 785 0; 0 0 10.8]; % kg*m2
            this.fuse.Front.CG = [0 0 -2.68];
            
            assert(det(this.fuse.Central.Inertia) ~= 0);
            assert(det(this.fuse.Front.Inertia) ~= 0);
            assert(det(this.fuse.Back.Inertia) ~= 0);
        end
        
        function plotEuler(this, t, y)
            subplot(3,1,1); hold on;
            plot(t, y(:,1)*180/pi, this.lineStyle)
            ylabel('\phi [deg]');

            subplot(3,1,2); hold on;
            plot(t, y(:,2)*180/pi, this.lineStyle)
            ylabel('\theta [deg]');

            subplot(3,1,3); hold on;
            plot(t, y(:,3)*180/pi, this.lineStyle)
            ylabel('\psi [deg]');
        end
        
        function plotSpeed(this, t, y)
            subplot(3,1,1); hold on;
            plot(t, y(:,4), this.lineStyle)
            ylabel('V_x [m]');

            subplot(3,1,2); hold on;
            plot(t, y(:,5), this.lineStyle)
            ylabel('V_y [m]');

            subplot(3,1,3); hold on;
            plot(t, y(:,6), this.lineStyle)
            ylabel('V_z [m]');
        end
    end
end
