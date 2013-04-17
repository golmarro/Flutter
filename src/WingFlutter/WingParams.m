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
        Xsp_p = 0.33         % [-] Shear center point position
        emptyWingXcg_p      = 0.01  % [-] CG position of empty wing (relatively to SSP)
        fuelXcg_p           = 0.06  % [-] CG of fuel (relatively to SSP)
        % Xcg_p = 0.01    % [-] Relatively to Xsp
        Xac_p = 0.125       % [-] TODO 0.25 should be calculated from xac = - CMalpha / CLalpha
        Ycg_p = 0.35        % [-]
        Yac_p = 0.7         % [-]
        
        plane               % reference to plane configuration class (PlaneParams)
        
        fuelLevel = 0.5     % [-]
    end
    
    properties(Constant)
        lbf_N = 4.44822162;
        ft_m = 0.3048;
        in_m = 0.0254;
        slug_kg = 14.5939029;
        alphaMax = 12*pi/180;
        
        fuelMaxMass         = 300   % [kg] about 400 liters of fuel
        wingEmptyMass       = 90    % [kg] single wing
        emptyWingItheta     = 7.56  % [kg*m2]
    end
	
    properties(GetAccess = 'public', SetAccess = 'public')    
        g = 9.81;           % m/s2
        % Structural data
        
        % mass =             165;           % kg {reduced to AC: F = m*h_dotdot)
        % Inertia approximation: m*C1^2/12
        %Itheta =        7.56;            % kg*m2
        % Itheta_0 =        11.34;            % kg*m2
        % Kphi =          870000;                  % [Nm/rad]
        Kh =            50000;                  % [Nm/rad]
        Ktheta =        36000;           % [Nm/rad]
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
        
        % Stale przyspieszeniomierzy
        % Tylko te polozone na koncu skrzydla
        Dleo =         -0.4           % m
        Dteo =          0.4           % m
    end
    
    properties(Dependent)
        CMalpha, CMalphadot, CMq, CMdelta
        Kphi
        shtheta
        Itheta      % kg * m2 - with respect to AC axis
        Itheta_0    % kg * m2 - with respect to CG axis
        omegah, omegatheta
        dampphi, damptheta
        mass        % kg static mass of wing dependant from fuel level
        m           % kg {reduced to AC: F = m*h_dotdot)
        S
        l        % AC position relatively to SSP
        Xcg_p, Xcg, Xac, Xsp, Ycg, Yac
        Iphi, Ipsi
    end
    
    properties(Dependent)   % Defined for Simulink model
        Inertia
    end
    
    methods
        function this = WingParams(plane)
            if exist('plane','var')
                this.plane = plane;
            end
        end
        
        % --------------------------------------- [getters]
        function val = get.CMalpha(this)
            val = this.CMalpha_50 + (this.Xsp_p - 0.5)*this.CLalpha;
        end
        function val = get.CMdelta(this)
            val = this.CMdelta_50 + (this.Xsp_p - 0.5)*this.CLdelta;
        end
        function val = get.CMalphadot(this)
            val = this.CMalphadot_50 + (this.Xsp_p - 0.5)*this.CLalphadot;
        end
        function val = get.CMq(this)
            val = this.CMq_50 + (this.Xsp_p - 0.5)*this.CLq;
        end
        function val = get.omegah(this)
            val = sqrt(this.Kh / this.m);
        end
        function val = get.omegatheta(this)
            val = sqrt(this.Ktheta / this.Itheta);
        end
        function val = get.dampphi(this)
            val = 2 * this.mass * this.zetah * this.omegah * this.Yac^2;
        end
        function val = get.damptheta(this)
            val = 2 * this.Itheta_0 * this.zetatheta * this.omegatheta;
        end
        function val = get.shtheta(this)
            val = this.mass * this.Xcg;
        end
        function val = get.Itheta(this)
            val = this.Itheta_0 + this.mass * this.Xcg^2;
        end
        function val = get.Itheta_0(this)
            val = this.emptyWingItheta * (1 + this.fuelLevel);
        end
        function val = getS(this)
            val = (this.C1 + this.C2)/2*this.L;  %[m2]
        end
        function val = get.l(this)
            val = (this.Xac_p - this.Xsp_p) * this.c;
        end
        function val = get.Kphi(this)
            val = this.Kh * this.Yac^2;
        end
        function val = get.m(this)
            val = this.Iphi / this.Yac^2 + (this.Ycg/this.Yac)^2 * this.mass;
        end
        function val = get.mass(this)
            val = this.wingEmptyMass + this.fuelLevel * this.fuelMaxMass /2;
        end
        function val = get.Iphi(this)
            val = this.mass*this.L^2/12;
        end
        function val = get.Ipsi(this)
            val = this.mass*this.L^2/12;
		end
		function val = get.Inertia(this)
			val = [this.Itheta_0 0 0; 0 this.Iphi 0; 0 0 this.Ipsi];
        end
        function val = get.Xcg_p(this)
            val = (this.emptyWingXcg_p * this.wingEmptyMass + this.fuelXcg_p * this.fuelMaxMass * this.fuelLevel/2)...
                   / (this.wingEmptyMass + this.fuelMaxMass * this.fuelLevel/2);
        end
        function val = get.Xcg(this)
            val = this.Xcg_p * this.c;
        end
        function val = get.Xac(this)
            val = this.Xac_p * this.c;
        end
        function val = get.Xsp(this)
            val = this.Xsp_p * this.c;
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
        
        function sys = getTurbulence(this, U0)
            % Turbulence from Bact document for speed 300 fps;
            alpha = 0.007;
            beta_p = 0.521;
            gamma_p = 0.533;
            L_t = 4.163;

            V = 300; % fps

            tmp = (2*pi*L_t/V)^2;
            gamma = gamma_p*tmp;
            beta = beta_p*tmp;

            K = 2*pi*sqrt(alpha*beta)/gamma;
            A = 2*pi/sqrt(beta);
            B = 2*pi/sqrt(gamma);

            % Transfer function
            %  w_g       s+A
            %  --- = K -------
            %  ni      (s+B)^2

            %wg = K*tf([1 A],[1 2*A A^2]);
            %wg_dot = K*tf([1 A 0],[1 2*A A^2]);
            %w_tf = [wg;wg_dot];

            % State-space model
            At = [0 1; -B^2 -2*B];
            Bt = [0;1];
            Ct = K*[A 1; -B^2 (A-2*B)];
            Dt = K*[0;1];
            
            % Conversion to metric units
            Ct = this.ft_m * Ct;
            Dt = this.ft_m * Dt;
            
            sys = ss(At,Bt,Ct,Dt);
            sys.StateName = ['turb1';'turb2'];
            sys.InputNames = 'turb in';
            sys.OutputName = ['w     [m/s]';
                              'w_dot [m/s]'];
        end
        
        function sys = getTurbulenceTF(this, U0)
            % Turbulence from Bact document for speed 300 fps;
            alpha = 0.007;
            beta_p = 0.521;
            gamma_p = 0.533;
            L_t = 4.163;

            V = 300; % fps

            tmp = (2*pi*L_t/V)^2;
            gamma = gamma_p*tmp;
            beta = beta_p*tmp;

            K = 2*pi*sqrt(alpha*beta)/gamma;
            A = 2*pi/sqrt(beta);
            B = 2*pi/sqrt(gamma);

            % Transfer function
            %  w_g       s+A
            %  --- = K -------
            %  ni      (s+B)^2

            wg = this.ft_m*K*tf([1 A],[1 2*A A^2]);
            wg_dot = this.ft_m*K*tf([1 A 0],[1 2*A A^2]);
            sys = [wg;wg_dot];
            
            sys.InputNames = 'turb in';
            sys.OutputNames = ['w     [m/s]';
                               'w_dot [m/s]'];
        end
    end    
end

