classdef WingParams < handle
    %WINGPARAMS Structural and aerodynamic parameters for flutter model
    %     _
    %     x                              leading edge
    %     <----+----------+------+----+---+
    %          1          |      |    |   0
    %                     |      |    |   
    %                     |<-----|    Xac
    %                     |      Xsp
    %                     Xcg
    % Xsp_p      - with reference to leading edge
    % Xcg_p      - with reference to Shear Center Xsp (+ aft)
    % Xac_p      - with reference to leading edge
    
    %
    %    /|                       L/L      y
    %    /+------o----------o------+------->
    %    /|     Ycg        Yac
    % Ycg_p      - with reference to wing mount (percent of L)
    % Yac_p      - with reference to wing mount (percent of L)
    
    properties(GetAccess = 'public', SetAccess = 'public')
        CLinverted
        Xsp_p = 0.33         % [-] Shear center point position
        Xcg_p = 0.01    % [-] Relatively to Xsp
        Xac_p = 0.125       % [-] TODO 0.25 should be calculated from xac = - CMalpha / CLalpha
        Ycg_p = 0.35        % [-]
        Yac_p = 0.7         % [-]
        CLSign = 1
        
        plane               % reference to plane configuration class (PlaneParams)
    end
    
    properties(Constant)
        lbf_N = 4.44822162;
        ft_m = 0.3048;
        in_m = 0.0254;
        slug_kg = 14.5939029;
        alphaMax = 12*pi/180;
    end
	
    properties(GetAccess = 'public', SetAccess = 'public')    
        g = 9.81;           % m/s2
        % Structural data
        
        mass =             165;           % kg {reduced to AC: F = m*h_dotdot)
        % Inertia approximation: m*C1^2/12
        %Itheta =        7.56;            % kg*m2
        Itheta =        11.34;            % kg*m2
        % Kpsi =          870000;                  % [Nm/rad]
        Kh =            700000;                  % [Nm/rad]
        Ktheta =        40000;           % [Nm/rad]
        %omegah =        21.01;          % rad/s
        %omegatheta =    32.72;          % rad/s
        zetah =         0.0014;         % -
        zetatheta =     0.001;          % -
        %shtheta =       0.0142;         % slug-ft
        %shdelta =       0.00288;        % slug-ft
        %sthetadelta =   0.00157;        % slug-ft2
        alpha0 =        0;              % rad
        
        % Aerodynamic data (relative to 50% of mean chord)
        CL0 =           0               % 1/rad
        CLalpha =       4.584;          % 1/rad
        CLalphadot =   -3.1064;         % 1/rad
        CLq =           2.5625;         % 1/rad
        CLdelta =       0.63;           % 1/rad
        
        CM0 =           0               % 1/rad
        CMalpha_50 =    1.490;          % 1/rad
        CMalphadot_50 =-2.6505;         % 1/rad
        CMq_50 =       -0.4035;         % 1/rad
        CMdelta_50 =   -0.0246;         % 1/rad
        
        CLdeltadot =    0;              % 1/rad
        CMdeltadot_50 = 0;              % 1/rad
    end
    
    properties %(Constant)
        C1 = 1.1;                          % [m]
        C2 = 0.4;                          % [m]
        L = 7;          % [m]
        c =             0.75;                 % [m]
        % l =            -0.175 * 1.33;      % ft
        
        % Stale przyspieszeniomierzy (actuators data)
        % Tylko te polozone na koncu skrzydla
        %Dleo =         -0.599           % ft
        %Dteo =          0.420           % ft
    end
    
    properties(Dependent)
        CMalpha, CMalphadot, CMq, CMdelta
        %Kh
        shtheta
        omegah, omegatheta
        m           % kg {reduced to AC: F = m*h_dotdot)
        S
        l        % AC position relatively to SSP
        Xcg, Xac, Ycg, Yac
    end
    
    properties(Dependent)   % Defined for Simulink model
        Inertia
    end
    
    methods
        function this = WingParams(string, plane)
            if exist('plane','var')
                this.plane = plane;
            end
            if exist('string','var')
                string = lower(string);

%                 if ~isempty(strfind(string,'metric'))
%                     this.convertToMetric();
%                 end
                
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
        end
        
        % --------------------------------------- [getters]
        function val = get.CMalpha(this)
            val = this.CMalpha_50 + (this.Xsp_p - 0.5)*this.CLalpha * this.CLSign;
        end
        function val = get.CMdelta(this)
            val = this.CMdelta_50 + (this.Xsp_p - 0.5)*this.CLdelta * this.CLSign;
        end
        function val = get.CMalphadot(this)
            val = this.CMalphadot_50 + (this.Xsp_p - 0.5)*this.CLalphadot * this.CLSign;
        end
        function val = get.CMq(this)
            val = this.CMq_50 + (this.Xsp_p - 0.5)*this.CLq * this.CLSign;
        end
        function val = get.omegah(this)
            val = sqrt(this.Kh / this.m);
        end
        function val = get.omegatheta(this)
            val = sqrt(this.Ktheta / this.Itheta);
        end
        function val = get.shtheta(this)
            val = this.m * this.Xcg;
        end
        function val = getS(this)
            val = (this.C1 + this.C2)/2*this.L;  %[m2]
        end
        function val = get.l(this)
            val = (this.Xac_p - this.Xsp_p) * this.c;
        end
%         function val = get.Kh(this)
%             val = this.Kpsi / this.Yac ^2;
%         end
        function val = get.m(this)
            val = this.Itheta / this.Xac^2 + (this.Xcg/this.Xac)^2 * this.mass;
        end
		function val = get.Inertia(this)
			val = [this.Itheta 0 0; 0 0 0; 0 0 0];
		end
        function val = get.Xcg(this)
            val = this.Xcg_p * this.c;
        end
        function val = get.Xac(this)
            val = this.Xac_p * this.c;
        end
        function val = get.Ycg(this)
            val = this.Ycg_p * this.L;
        end
        function val = get.Yac(this)
            val = this.Yac_p * this.L;
        end
        function val = get.S(this)
            val = (this.C1 + this.C2)/2*this.L;
        end
        
        % --------------------------------------- [setters]
        function set.Xcg(this, val)
            this.Xcg_p = val / this.c;
        end
        function set.Xac(this, val)
            this.Xac_p = val / this.c;
        end
        function set.Ycg(this, val)
            this.Ycg_p = val / this.L;
        end
        function set.Yac(this, val)
            this.Yac_p = val / this.L;
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
        
%         function convertToMetric(this)
%             slug_kg = this.slug_kg;
%             ft_m = this.ft_m;
%             lbf_N = this.lbf_N;
%             
%             this.g = this.g             * ft_m;              % ft/s2 -> m/s2
%             
%             this.m = this.m             * slug_kg;           % slug -> kg
%             this.Itheta = this.Itheta   * slug_kg * ft_m^2;  % slug*ft2 -> kg*m2
%             this.Kh = this.Kh           * lbf_N / ft_m;      % lb/ft -> N/m
%             this.Ktheta = this.Ktheta   * lbf_N * ft_m;      % ft-lb -> N*m
%             %this.shtheta = this.shtheta * slug_kg * ft_m;    % slug-ft -> kg*m
%             %this.shdelta = this.shdelta * slug_kg * ft_m;    % slug-ft -> kg*m
%             %this.sthetadelta = this.sthetadelta * slug_kg * ft_m^2;   % slug-ft2 -> kg*m2
% 
%             this.S = this.S            * ft_m^2;            % ft2 -> m2
%             this.c = this.c            * ft_m;              % ft  -> m
%             %this.l = this.l            * ft_m;              % ft  -> m
%         end
    end
    
end

