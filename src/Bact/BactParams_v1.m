classdef BactParams < handle
    %BACTPARAMS Summary of this class goes here
    %   Detailed explanation goes here
    properties(GetAccess = 'public', SetAccess = 'public')
        units = 'english'
        CLinverted
    end
    
    properties(Constant)
        lbf_N = 4.44822162;
        ft_m = 0.3048;
        in_m = 0.0254;
        slug_kg = 14.5939029;
    end
    
    properties(GetAccess = 'public', SetAccess = 'public')    
        g = 32.2;           % ft/s2
        % Structural data
        m =             6.08;           % slug
        Itheta =        2.8;            % slug*ft2
        Kh =            2686;           % lb/ft
        Ktheta =        3000;           % ft-lb
        omegah =        21.01;          % rad/s
        omegatheta =    32.72;          % rad/s
        zetah =         0.0014;         % -
        zetatheta =     0.001;          % -
        shtheta =       0.0142;         % slug-ft
        %shdelta =       0.00288;        % slug-ft
        %sthetadelta =   0.00157;        % slug-ft2
        alpha0 =        0;              % rad
        
        % Aerodynamic data
        CLalpha =       4.584;          % 1/rad
        CMalpha =       1.490;          % 1/rad
        CLalphadot =   -3.1064;         % 1/rad
        CLq =           2.5625;         % 1/rad
        CMalphadot =   -2.6505;         % 1/rad
        CMq =          -0.4035;         % 1/rad
        CLdelta =       0.63;           % 1/rad
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
    
    methods
        function this = BactParams(string)
            if exist('string','var')
                string = lower(string);

                if ~isempty(strfind(string,'metric'))
                    this.units = 'metric';
                elseif ~isempty(strfind(string,'english'))
                    this.units = 'english';
                end

                if ~isempty(strfind(string,'clinverse'))
                    % Inverted lift - alpha relation used in [1]
                    this.CLinverted = 'yes';
                    this.inverseCL();
                else
                    this.CLinverted = 'no';
                end
            end
        end
        
        function set.units(this, val)
            if strcmp(this.units, val)
                return
            end
            if strcmp(val, 'metric')
                this.convertToMetric();
            elseif strcmp(val, 'english')
                assert(0, 'Not implemented');
            end
            this.units = val;
        end
    end
    
    methods %TODO private
        function inverseCL(this)
            this.CLalpha =      -this.CLalpha;
            this.CLalphadot =   -this.CLalphadot;
            this.CLq =          -this.CLq;
            this.CLdelta =      -this.CLdelta;
            this.CLdeltadot =   -this.CLdeltadot;
        end
        
        function convertToMetric(this)
            slug_kg = this.slug_kg;
            ft_m = this.ft_m;
            lbf_N = this.lbf_N;
            
            this.g = this.g             * ft_m;              % ft/s2 -> m/s2
            
            this.m = this.m             * slug_kg;           % slug -> kg
            this.Itheta = this.Itheta   * slug_kg * ft_m^2;  % slug*ft2 -> kg*m2
            this.Kh = this.Kh           * lbf_N / ft_m;      % lb/ft -> N/m
            this.Ktheta = this.Ktheta   * lbf_N * ft_m;      % ft-lb -> N*m
            this.shtheta = this.shtheta * slug_kg * ft_m;    % slug-ft -> kg*m
            this.shdelta = this.shdelta * slug_kg * ft_m;    % slug-ft -> kg*m
            this.sthetadelta = this.sthetadelta * slug_kg * ft_m^2;   % slug-ft2 -> kg*m2

            this.S = this.S            * ft_m^2;            % ft2 -> m2
            this.c = this.c            * ft_m;              % ft  -> m
            this.l = this.l            * ft_m;              % ft  -> m
        end
    end
    
end

