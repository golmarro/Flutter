classdef BactParams < handle
    %BACTPARAMS Structural and aerodynamic parameters for flutter model
    %     _
    %     x                              leading edge
    %     <----+----------+------+-------+
    %          1          |      |       0
    %                     |      |
    %                     |      refpoint [%]
    %                     cgPos
    % refPoint - with reference to leading edge
    % cgPos    - with reference to refPoint
    
    properties(GetAccess = 'public', SetAccess = 'public')
        units = 'english'
        CLinverted
        refPoint = 0.5      % Aero. coeff. calculated relatively to mid-chord [%]
        cgPos = 0.001756
        CLSign = 1
		% for simulink:
		Xcg = 0.001756
		Ycg = 1
		Xsp = 0
		Ysp = 1
		
		damph
		damptheta
    end
	
	properties(Constant)	% For Simulink simulation
		L = 2;
		C1 = 1.33;
		C2 = 1.33;
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
        %shtheta =       0.0142;         % slug-ft
        %shdelta =       0.00288;        % slug-ft
        %sthetadelta =   0.00157;        % slug-ft2
        alpha0 =        0;              % rad
        
        % Aerodynamic data (relative to 50% of mean chord)
		CL0 =           0
		CM0 =           0
        CLalpha =       4.584;          % 1/rad
        CMalpha_50 =    1.490;          % 1/rad
        CLalphadot =   -3.1064;         % 1/rad
        CLq =           2.5625;         % 1/rad
        CMalphadot_50 =-2.6505;         % 1/rad
        CMq_50 =       -0.4035;         % 1/rad
        CLdelta =       0.63;           % 1/rad
        CLdeltadot =    0;              % 1/rad
        CMdelta_50 =   -0.0246;         % 1/rad
        CMdeltadot_50 = 0;              % 1/rad
        S =             3.55;           % ft2
        c =             1.33;           % ft
        % l = -0.175 %c
        l =            -0.175 * 1.33;      % ft
        
        % Stale przyspieszeniomierzy (actuators data)
        % Tylko te polozone na koncu skrzydla
        Dleo =         -0.599           % ft
        Dteo =          0.420           % ft
    end
    
    properties(Dependent)
        CMalpha
        CMalphadot
        CMq
        CMdelta
        shtheta
		Inertia
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
                    this.CLSign = -1;
                    this.inverseCL();
                else
                    this.CLinverted = 'no';
                    this.CLSign = 1;
                end
            end
			
			%this.damph = 2 * this.zetah * sqrt(this.m * this.Kh);
            %this.damptheta = 2 * this.zetatheta * sqrt(this.Itheta * this.Ktheta);
        end
        
        % --------------------------------------- [get]
        function val = get.CMalpha(this)
            val = this.CMalpha_50 + (this.refPoint - 0.5)*this.CLalpha * this.CLSign;
        end
        function val = get.CMdelta(this)
            val = this.CMdelta_50 + (this.refPoint - 0.5)*this.CLdelta * this.CLSign;
        end
        function val = get.CMalphadot(this)
            val = this.CMalphadot_50 + (this.refPoint - 0.5)*this.CLalphadot * this.CLSign;
        end
        function val = get.CMq(this)
            val = this.CMq_50 + (this.refPoint - 0.5)*this.CLq * this.CLSign;
        end
        
        function val = get.shtheta(this)
            val = this.m * this.cgPos * this.c;
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
		
		function val = get.Inertia(this)
			val = [this.Itheta 0 0; 0 2 0; 0 0 2];
		end
        
        function compare(this, other)
            fields = fieldnames(this);
            fprintf('\t\tthis\tother\n');
            for i = 1:length(fields);
                try
                    val = other.(fields{i});
                    thisVal = this.(fields{i});
                    fprintf('%s: \t%f\t%f\n', fields{i}, thisVal(1), val(1));
                catch me
                end
            end
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
            %this.shtheta = this.shtheta * slug_kg * ft_m;    % slug-ft -> kg*m
            %this.shdelta = this.shdelta * slug_kg * ft_m;    % slug-ft -> kg*m
            %this.sthetadelta = this.sthetadelta * slug_kg * ft_m^2;   % slug-ft2 -> kg*m2
			
			this.damph = 2 * this.zetah * sqrt(this.m * this.Kh);
            this.damptheta = 2 * this.zetatheta * sqrt(this.Itheta * this.Ktheta);

            this.S = this.S            * ft_m^2;            % ft2 -> m2
            this.c = this.c            * ft_m;              % ft  -> m
            this.l = this.l            * ft_m;              % ft  -> m
        end
    end
    
end

