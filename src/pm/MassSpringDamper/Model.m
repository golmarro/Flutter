classdef Model < handle
    %MODEL Mass-Damper-Spring model with uncertainty
    %   Model taken from "Robust Control Design with Matlab" 
    %   by P. Hr. Petkov and M. M. Konstantinov (chapter 8)
    
    properties(Constant)
        m = 3;      % mass, damping and spring constant
        c = 1;
        k = 2;

        %pm = 0.4;   % percentage variations of parameters
        %pc = 0.2;
        %pk = 0.3;
        
        pm = 0.6;   % percentage variations of parameters
        pc = 0.6;
        pk = 0.4;
    end
    
    properties
        Wp          % Weighting function (performance)
        Wu          % Control signal weighting function
        Wunc  = []  % Multiplicative Uncertainty Weight
        Khinf       % Result of HinfSynthesis
        Clw         % Closed-loop system with weighting functions
        
        deltaNom = [0 0 0]   % Delta for nominal plant - if we want to use other than average
    end
    
    properties(Dependent)
        Gn          % Nominal plant
        Gdelta      % Model in P-Delta configuration (created manually)
        Gdelta2     % Model in P-Delta configuration (using uncertain atoms)
        Ganal       % Model for performance analysis
        % Weighted model (for controller synthesis)
        Gsynth      % using system interconnection script (sysic)
        Gsynth2     % created manually
        Gsynth3     % created using augw function
        GsynthStab  % added robust stability criteria
        Cl          % Closed-loop system
    end
    
    methods
        function this = Model()
            this.Wp = 0.95*ss(tf([1 1.8 10],[1 8 0.01]));
            this.Wu = 0.01*ss(tf(1,1));
        end
        
        function val = get.Gn(this)
            m = this.m * (1 + this.deltaNom(1)*this.pm);
            c = this.c * (1 + this.deltaNom(2)*this.pc);
            k = this.k * (1 + this.deltaNom(3)*this.pk);
            
            A = [-c/m -k/m; 1 0];
            B = [1/m; 0];
            C = [0 1];
            D = 0;
            val = ss(A,B,C,D);
        end
        
        function val = get.Gdelta(this)
            A = [0 1; -this.k/this.m -this.c/this.m];
            B1 = [0 0 0; -this.pm -this.pc/this.m -this.pk/this.m];
            B2 = [0; 1/this.m];
            C1 = [-this.k/this.m -this.c/this.m; 0 this.c; this.k 0];
            C2 = [1 0];
            D11 = [-this.pm -this.pc/this.m -this.pk/this.m; 0 0 0; 0 0 0];
            D12 = [1/this.m ; 0 ; 0];
            D21 = [0 0 0];
            D22 = 0;
            val = ss(A,[B1 B2],[C1; C2],[D11 D12; D21 D22]);
        end
        
        function G = get.Ganal(this)
            Gnom = this.Gn;
            systemnames = 'Gnom';
            inputvar = '[dist; control]';
            outputvar = '[Gnom+dist; control]';
            input_to_Gnom = '[control]';
            cleanupsysic = 'yes';
            sysoutname = 'G';
            sysic
        end
        
        %                                      | d
        %                                      |
        %   r         +-----+   u   +------+   v    +------+
        % ---->o----->|  K  |------>|  Gn  |---o--->|  Wp  |---> ep
        %      ^      +-----+   |   +------+   |    +------+
        %      |                |              |    +------+
        %      |                ---------------+--->|  Wu  |---> eu
        %      --------------------------------/    +------+
        
        function G = get.Gsynth(this)
            Gnom = this.Gn;
            Wp = this.Wp;
            Wu = this.Wu;

            systemnames = 'Gnom Wu Wp';
            inputvar = '[dist; control]';
            outputvar= '[Wp; Wu; Gnom + dist]';
            input_to_Gnom = '[control]';
            input_to_Wp = '[Gnom + dist]';
            input_to_Wu = '[control]';
            cleanupsysic = 'yes';
            sysoutname = 'G';
            sysic
        end
        
        function G = get.Gsynth2(this)
            Gnom = this.Gn;
            Wp = this.Wp;
            Wu = this.Wu;
            Au = Wu.A;
            Bu = Wu.B;
            Cu = Wu.C;
            
            A1 = [Gnom.A              this.z(Gnom.A,Wp.A)  this.z(Gnom.A,Wu.A)];
            A2 = [Wp.B*Gnom.C         Wp.A                 this.z(Wp.A,Wu.A)  ];
            A3 = [this.z(Wu.A,Gnom.A) this.z(Wu.A,Wp.A)    Wu.A              ];
            A  = [A1;A2;A3];
            
            B1 = [this.z(Gnom.B,Wp.B)  Gnom.B         ];
            B2 = [Wp.B                 this.z(Wp.B,Wu.B)];
            B3 = [this.z(Wu.B,Wp.B)    Wu.B           ];
            B = [B1;B2;B3];
            
            C = [Wp.D*Gnom.C          Wp.C                this.z(Wp.C,Wu.C);
                 this.z(Wu.C,Gnom.C)  this.z(Wu.C,Wp.C)   Wu.C      ;
                 Gnom.C               this.z(Gnom.C,Wp.C) this.z(Gnom.C,Wu.C)];
            D = [Wp.D  0;  0  Wu.D;  1  0];
            
            G = ss(A,B,C,D);
        end
        
        function G = get.Gsynth3(this)
            G = augw(this.Gn, this.Wp, this.Wu);
        end
             
        function HinfSynthesis(this)
            [K,CL,GAM,INFO] = hinfsyn(this.Gsynth,1,1,'display','on');
            this.Khinf = K;
            this.Clw = CL;
            this.Clw.OutputName = ['Wp';'Wu'];
            this.Clw.InputName = 'u';
            
            this.checkStability;
        end
        
        %                                      | d
        %                                      |
        %   r         +-----+   u   +------+   v    +------+
        % ---->o----->|  K  |------>|  Gn  |---o--->|  Wp  |---> ep
        %      ^      +-----+   |   +------+   |    +------+
        %      |                |              |    +------+
        %      |                ---------------+--->|  Wu  |---> eu
        %      --------------------------------/    +------+
        
        function G = get.GsynthStab(this)
            if isempty(this.Wunc)
                error('Uncertainty weighting function undefined this.Wunc, use unstructUncert() first');
            end
            G = augw(this.Gn, this.Wp, this.Wu, this.Wunc);
        end
                
        function HinfSynthesisStab(this)
            [K,CL,GAM,INFO] = hinfsyn(this.GsynthStab,1,1,'display','on');
            this.Khinf = -K;
            this.Clw = CL;
            this.Clw.OutputName = ['Wp  ';'Wu  ';'Wunc'];
            this.Clw.InputName = 'u';
            
            this.checkStability;
        end
        
        function checkStability(this)
            % Brute force
            % Check robust stability
            stable = 1;
            delta = this.getDelta();
            for i=1:size(delta,1)
                Gpert = lft(diag(delta(i,:)),this.Gdelta);
                cl = feedback(Gpert,this.Khinf,1);
                if sum(real(eig(cl))>0) > 0
                    fprintf('Unstable for: %f, %f, %f\n', delta(i,1), delta(i,2), delta(i,3));
                    stable = 0;
                end
%                 sigma(Gpert);
%                 hold on
            end
            if stable == 1
                fprintf('Closed-loop seems to be robustly stable\n');
            end
        end
        
        function val = get.Cl(this)
            % assert(this.Khinf ~= [])
            val  = feedback(this.Gn, this.Khinf, 1);
        end
        
        function [Wu Guns] = unstructUncert(this, order, plot)
            if ~exist('order','var')
                order = 2;
            end
            pert = this.getDelta;
            
            % Dodajmy pierwszy model
            Gi = lft(diag(pert(1,:)), this.Gdelta);
            Garray = stack(1,Gi);
            for i = 2:size(pert,1)
                Gi = lft(diag(pert(i,:)), this.Gdelta);
                Garray = stack(1,Garray,Gi);
            end

            % Funkcja ucover domysle stosuje niepewnosc multiplikatywna na wejsciu -
            % dla ukladow jednowymiarowych nie ma znaczenia czy na wejsciu czy na
            % wyjsciu
            [Guns info] = ucover(Garray, this.Gn, order);
            Wu = info.W1;
            this.Wunc = info.W1;
            if exist('plot','var')
                if plot == 1
                    %sigma(this.Gn, 'b')
                    [svWunc w] = sigma(this.Wunc);
                    svGn = sigma(this.Gn, w);
                    semilogx(w, mag2db(svGn(1,:)),'b');
                    hold on;
                    %sigma(this.Gn + this.Gn*this.Wunc, 'r--')
                    semilogx(w, mag2db(svGn(1,:) + svWunc(1,:).*svGn(1,:)), 'r--');
                    for i = 1 : length(Garray)
                        sigma(Garray(:,:,i),'k:');
                    end
                    legend('G_n(s)','(I + w_o(s))(G_n(s))','rodzina G_r');
                elseif plot == 2
                    % ------------------------ error plot
                    [svWunc w] = sigma(this.Wunc);
                    svGn = sigma(this.Gn, w);
                    semilogx(w, mag2db(svWunc(1,:)), 'r--');
                    hold on;
                    for i = 1 : length(Garray)
                        svGr = sigma(Garray(:,:,i),w);
                        semilogx(w, mag2db((svGr(1,:) - svGn(1,:))./svGn(1,:)), 'k:');
                    end
                    legend('w_o(s)','(G_r(s) - G_n(s))/G_n(s)');
                end
                % Gorny i dolny przedzial w ktorym moze znalezc sie Gp:
%                 [sv w] = sigma(this.Gn + this.Gn*this.Wunc);
%                 frGn = freqresp(this.Gn, w);
%                 frWunc = freqresp(this.Wunc, w);
%                 frGn = squeeze(frGn);
%                 frWunc = squeeze(frWunc);
%                 plot(w, mag2db(abs(frGn)), 'g--');
%                 plot(w, mag2db((1+abs(frWunc)) .* abs(frGn)),'b--');
%               % Dolny przedzial traci sens dla w(jw) > 2 bo Gp moze byc
%               % zero a wrecz ujemne (przeciwna faza)
%                 plot(w, mag2db((1-abs(frWunc)) .* abs(frGn)),'b--');
            end
        end
    end
    
    methods(Static)
        function z = z(a,b)
            z = zeros(size(a,1), size(b,2));
        end
        
        function delta = getDelta()
            delta = zeros(27,3);
            i = 1;
            for a = [-1 0 1]
                for b = [-1 0 1]
                    for c = [-1 0 1]
                        delta(i,:) = [a,b,c];
                        i = i+1;
                    end
                end
            end
        end
    end
    
end