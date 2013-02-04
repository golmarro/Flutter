classdef PlaneParams < handle
    %PLANEPARAMS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        fuelLevel = 0.5             % [-]
        payloadLevel = 0            % [-]
        wingParams = []
    end
    
    properties(Constant)
        fuelMaxMass         = 300   % [kg] about 400 liters of fuel
        payloadMaxMass      = 200   % [kg]
        
        wingEmptyMass       = 90    % [kg] single wing
        fuselageEmptyMass   = 100   % [kg]
        engineMass          = 80    % [kg]
        avionicsMass        = 110   % [kg]
        stabMass            = 20    % [kg] single stabilizer
        
        emptyWingXcg_p      = 0.01  % [-] CG position of empty wing (relatively to SSP)
        fuelXcg_p           = 0.06  % [-] CG of fuel (relatively to SSP)
        emptyWingItheta     = 7.56  % [kg*m2]
    end
    
    properties(Dependent)
        wingMass
        fuselageMass
        totalMass
        massPerWing
        wingXcg_p          % CG position of wing (relatively to SSP)
        wingItheta
    end
    
    methods
        function this = PlaneParams(wingParams)
            if exist('wingParams','var')
                this.wingParams = wingParams;
                this.setParams();
            end
        end
        
        function val = get.wingMass(this)
            val = this.wingEmptyMass + this.fuelLevel * this.fuelMaxMass /2;
        end
        function val = get.wingItheta(this)
            val = this.emptyWingItheta * (1 + this.fuelLevel);
        end
        function val = get.fuselageMass(this)
            val = this.fuselageEmptyMass + this.engineMass + this.avionicsMass...
                + this.payloadLevel * this.payloadMaxMass;
        end
        function val = get.totalMass(this)
            val = 2 * this.wingMass + 2 * this.stabMass + this.fuselageMass;
        end
        function val = get.massPerWing(this)
            val = this.stabMass + this.fuselageMass / 2;
        end
        function val = get.wingXcg_p(this)
            val = (this.emptyWingXcg_p * this.wingEmptyMass + this.fuelXcg_p * this.fuelMaxMass * this.fuelLevel/2)...
                   / (this.wingEmptyMass + this.fuelMaxMass * this.fuelLevel/2);
        end
        
        function set.fuelLevel(this, val)
            this.fuelLevel = val;
            this.setParams();
        end
        function set.payloadLevel(this, val)
            this.payloadLevel = val;
            this.setParams();
        end
        
        function setParams(this)
            if ~isempty(this.wingParams)
                this.wingParams.Xcg_p = this.wingXcg_p;
                this.wingParams.mass = this.wingMass;
                this.wingParams.Itheta_0 = this.wingItheta;
                %this.wingParams.Xcg_p = this.wingXcg_p;
                %this.wingParams.Xcg_p = this.wingXcg_p;
            end
        end
        
        function disp(this)
            fprintf('Plane configuration:\n');
            fprintf('\tTotal mass:           %f kg\n',this.totalMass);
            fprintf('\tWing mass:            %f kg\n',this.wingMass);
            fprintf('\tWing Xcg:             %f %%\n',this.wingXcg_p*100);
            fprintf('\tWing Itheta:          %f kg\n',this.wingItheta);
            %fprintf('\t:            %f kg\n',this.);
            %fprintf('\t:            %f kg\n',this.);
            fprintf('\tFuselage mass:        %f kg\n',this.fuselageMass);
            fprintf('\tFuel level:           %f %%\n',this.fuelLevel*100);
            fprintf('\tPayload level:        %f %%\n',this.payloadLevel*100);
        end
    end
    
end

