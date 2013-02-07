classdef StabParams < WingParams
    properties(GetAccess = 'public', SetAccess = 'public')
        stabAlpha0 = -4 * pi/180;
    end
    
    methods
        function this = StabParams()
            this.mass = 20;
            this.Itheta_0 = 1.5;
            % --------------------------- [dimensions] ----------- 
            this.c = 0.64;                          % [m]
            this.C1 = 0.64;                          % [m]
            this.C2 = 0.64;                          % [m]
            this.L = 2.5;                           % [m]
        end
    end    
end

