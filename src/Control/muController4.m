function [K descr] = muController4( wing , verbose, delta_max_p, uncert)
    % G - full model
    clear
    %% Get full model for flutter speed
    if ~exist('wing','var')
        params = WingUnc;
        %wing = WingFlutter(params);
        wing = WingFlutter();
        %wing.U0 = wing.getFlutterSpeed();
        %wing.U0 = params.U0_unc;
        wing.U0 = ureal('U0',95,'Range',[90, 100]);
    end
    
    G = wing.getLinearModel();
    G = simplify(G,'full');
    fprintf('###################################### mu controller\n');
    disp(G);
    
    if ~exist('Number','var')
        Number = 1;
    end
    if ~exist('delta_max_p','var')
        delta_max_p = 1;
    end
    if ~exist('verbose','var')
        verbose = 0;
    end
    if ~exist('uncert','var')
        uncert = 2;
    end
    
    h_max = 0.32;
    theta_max = 3.56*pi/180;
    delta_max = 45*pi/180;

    %% Analysis model P
    P = G([1,2,8,9],[2 1 1]);
    fprintf('Inputs:\n');
    disp(P.InputName)
    fprintf('Outputs:\n')
    disp(P.OutputName)
    P.InputName = ['turbule';'delta_p';'delta_c'];

    %% Add weights - synthesis model Pw
    %Wu = 1/(delta_max*0.3);
    Wu = 1/(delta_max*delta_max_p);
    washout = tf([1 0],[1 4.19]);
    Wp = [washout/h_max 0;
          0   washout/theta_max];
    Wdelta_p = tf(0.7*delta_max,[1/(3*2*pi)  1]);
    
    %% Uncertainty model
%     Vf = wing.getFlutterSpeed;
%     
%     if uncert == 1
%         [Wunc W2 Gunc] = wing.addUncert(3, 0, 0.5, 5500, Vf*[0.8 1 1.4]);
%     elseif uncert == 2
%         [Wunc W2 Gunc] = wing.multUncert(3, 0, 0.5, 5500, Vf*[0.8 1 1.4]);
%     end
    %%
%     WuncRef = eye(2)*Wunc;
    
    %%
    
%     Wunc = 0.02*WuncRef;
    Wunc = [h_max 0; 0 theta_max];
    systemnames = 'P Wp Wu Wdelta_p Wunc';
    inputvar = '[n1;n2;turb;delta_p;delta_c]';
    outputvar = '[Wp;Wu;Wunc;n1+P(3);n2+P(4)]';
    input_to_Wdelta_p = '[delta_p]';
    input_to_P = '[turb;Wdelta_p;delta_c]';
    input_to_Wp = '[P(1:2)]';
    input_to_Wu = '[delta_c]';
    input_to_Wunc = '[P(3:4)]';
    %outputnames = '[zp{2};zu;h_dd;theta_dd]';
    Pw = sysic;

    %Pw = augw(P,Wp, Wu, []);

    %% Synthesis
    opt = dkitopt('DisplayWhileAutoIter','on','NumberOfAutoIterations',2);
    
    [K, clp, bnd, dkinfo] = dksyn(Pw, 2, 1, opt);
%%
    K.InputName = P.OutputName(3:4);
    K.OutputName = G.inputName(1);
    
    fprintf('Khinf inputs:\n');
    disp(K.InputName)
    fprintf('Khinf outputs:\n')
    disp(K.OutputName)
    
    
    if Number == 1
        descr = 'Hinf 1';
        
    elseif Number == 2
        descr = 'Hinf 2';
    end
    
    %K = -K;
    
    %%
    if verbose == 1
        %figure; hold on;
        %
        %semilogx(w,sv(1,:));
        %semilogx(w(1,[1 end]), [gamma gamma]);
        
        % Szukamy aktywnego ograniczenia
        figure; hold on;
        %sigma(CL);
        sigma(zpk([],[],gamma));
        sigma(CL(1,:))
        sigma(CL(2,:))
        sigma(CL(3,:))
        legend('gamma', 'CL(1,:) h', 'CL(2,:) theta', 'CL(3,:) u');
        
        [sv w] = sigma(CL);
        [sv_max i] = max(sv(1,:));
        w_max = w(i);
        fprintf('max(sigma) = %f, w_max = argmax(sigma) = %f, gamma = %f', sv_max, w_max, gamma);
    end
    
    %% SVD analysis
    if verbose == 1
        CL.InputName = {'n1','n2','\eta','\delta_p'};
        CL.OutputName = {'z_{p1} = W_h(s) h','z_{p2} = W_\theta(s) \theta', 'z_u = W_u(s) \delta_c','zr1','zr2'};
        SvdAnalysis(CL);
        subplot(2,1,1);
        title('Analiza SVD uk³adu zamkniêtego');
        ylabel('Wektor wyjœciowy');
        xlabel('\omega [rad/s]');
        subplot(2,1,2);
        xlabel('\omega [rad/s]');
        title('');
        ylabel('Wektor wejœciowy');
    end
    
    % print -dpsc2 ../doc/Ilustracje/SvdHinf1.ps
    % print -depsc2 ../doc/Ilustracje/SvdHinf1.eps
    
    %% Vfcl
    %ana = LinearAnalysis(Khinf);
    %ana.getFlutterSpeed(90)
end

