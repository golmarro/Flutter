classdef HinfController < handle
    properties
        h_max = 0.1;
        theta_max = 3*pi/180;
        delta_max = 45*pi/180;
        %Khinf = []
        OL
    end
    
    properties(Dependent)
        P       % Model for analysis (only necessary signals)
        Pw      % Model for synthesis (with weights)
        CL
        Khinf
    end
    
    methods
        function this = HinfController(sys)
            if ~exist('sys','var')
                wing = WingFlutter;
                this.OL = wing.getLinearModel();
                wing.U0 = wing.getFlutterSpeed();
            else
                this.OL = sys;
            end
        end

        % Extract model for synthesis from full model:
        %  
        %    turb       +--------+ 
        %  ------------>|        |
        %    delta_p    |        | -----> z  (performance signal)
        %  ------------>+-o->    |
        %    delta_c    | |      |
        %  ------------>+-+      | -----> y  (measurement signal)
        %               |        |
        %               +--------+
%         function val = get.P(this)
%             % in:  delta_c, turb
%             % out: h theta h_ddot theta_ddot
%             val = this.OL([1 2 6 7],:); 
%         end
        
        function val = get.P(this)
            s2 = this.OL([1:5, 8:9],[1 1]);
            A = s2.A(1:6,1:6);
            B = s2.B(1:6,:);
            C = s2.C(:,1:6);
            D = s2.D;

            G = ss(A,B,C,D);
            G.InputName = ['delta_p';this.OL.InputName(1)];
            G.StateName = s2.StateName(1:6);
            G.OutputName = s2.OutputName();
            val = G([1 2 6 7],:); % h theta h_ddot theta_ddot
        end
        
        function val = get.Pw(this)
            Wu = this.delta_max*0.3;
            Wp = diag([this.h_max this.theta_max 0 0]);
            val = augw(this.P,Wp, Wu, []);
        end
        
        function K = get.Khinf(this)
            
            [K, CL] = hinfsyn(this.Pw, 2, 1, 'display', 'on');
            %Khinf = -Khinf;
            %Khinf.InputName = G.OutputName(3:4);
            %Khinf.OutputName = G.InputName(2);
            
            %cl = feedback(G,Khinf,2, 3:4);
        end
        
        function val = get.CL(this)
            if isempty(this.Khinf)
                warning('Controller not defined');
                return
            end
            
            val = feedback(this.P, this.Khinf,2,3:4, 1);
        end
        
        function controlSignalAnalysis(this)
            if isempty(this.Khinf)
                warning('Controller not defined');
                return
            end
            
            aol = LinearAnalysis([0 0 0 0]);
            cl = feedback(this.ol, this.Khinf, 1, 8:9, 1);
            
            
        end
        
        function svdAnalysis(this)
            % Get some initial range for omega:
            [sv w] = sigma(this.Clw);
            % Get a series of transfer function matrices Clw(jw)
            fr = freqresp(this.Clw);
            for i = 1:size(fr,3);
                [u s v] = svd(fr(:,:,i));
            end
        end
    end
end

