classdef StabParams < WingParams
    properties(GetAccess = 'public', SetAccess = 'public')
        stabAlpha0 = -4 * pi/180;
    end
    
    properties(Dependent)
        rightOrient = [stabAlpha0 0 -30]
        leftOrient = [stabAlpha0 0 210]
    end
    
    properties(Constant)
        %wingEmptyMass = 20;
    end
    
    methods
        function this = StabParams()
            %this.mass = 20;
            %this.Itheta_0 = 1.5;
            this.fuelLevel = 0;
            this.wingEmptyMass = 20;
            this.emptyWingItheta = 1.5;
            this.emptyWingXcg_p = 0;
            
            % --------------------------- [dimensions] ----------- 
            this.c = 0.64;                          % [m]
            this.C1 = 0.64;                          % [m]
            this.C2 = 0.64;                          % [m]
            this.L = 2.5;                           % [m]
        end

        % ------------------------------- [getters] --------------
        % Remember the order of rotations
        function val = get.rightOrient(this)
            val = [this.stabAlpha0 0 -30];
        end
        function val = get.leftOrient(this)
            val = [this.stabAlpha0 0 210];
        end
    end
end

