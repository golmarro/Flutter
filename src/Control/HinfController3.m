function [Khinf descr] = HinfController3( G , Number, verbose, delta_max_p)
    % G - full model
    %% Get full model for flutter speed
    if ~exist('G','var')
        wing = WingFlutter;
        wing.U0 = wing.getFlutterSpeed();
        G = wing.getLinearModel();
    end
    if ~exist('Number','var')
        Number = 1;
    end
    if ~exist('delta_max_p','var')
        delta_max_p = 0.3;
    end
    if ~exist('verbose','var')
        verbose = 0;
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
    
    if Number == 1
        % ----------------------------------------- Hinf #1
        %Wdelta_p = zpk([], [],0.7);
        Wdelta_p = 0.7*delta_max;
    elseif Number == 2
        % ----------------------------------------- Hinf #2
        %Wdelta_p = zpk([],-3*2*pi,0.7);
        Wdelta_p = tf(0.7*delta_max,[1/(3*2*pi)  1]);
    end

    systemnames = 'P Wp Wu Wdelta_p';
    inputvar = '[turb;delta_p;delta_c]';
    outputvar = '[Wp;Wu;P(3:4)]';
    input_to_Wdelta_p = '[delta_p]';
    input_to_P = '[turb;Wdelta_p;delta_c]';
    input_to_Wp = '[P(1:2)]';
    input_to_Wu = '[delta_c]';
    %outputnames = '[zp{2};zu;h_dd;theta_dd]';
    Pw = sysic;

    %Pw = augw(P,Wp, Wu, []);

    %% Synthesis
    [Khinf, CL, gamma, info] = hinfsyn(Pw, 2, 1, 'display', 'on');

%     if exist('verbose','var')
%         cl = feedback(P, Khinf,1,3:4, 1);
% 
%         systemnames = '[P Khinf]';
%         inputvar = '[turb;delta_p]';
%         outputvar = '[P(1:2);Khinf]';
%         input_to_P = '[turb;delta_p;Khinf]';
%         input_to_Khinf = '[P(3:4)]';
%         cl2 = sysic;
%     end
%%
    Khinf.InputName = P.OutputName(3:4);
    Khinf.OutputName = G.inputName(1);
    
    fprintf('Khinf inputs:\n');
    disp(Khinf.InputName)
    fprintf('Khinf outputs:\n')
    disp(Khinf.OutputName)
    
    
    if Number == 1
        descr = 'Hinf 1';
        
    elseif Number == 2
        descr = 'Hinf 2';
    end
    
    Khinf = -Khinf;
    
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
        CL.InputName = {'\eta','\delta_p'};
        CL.OutputName = {'z_{p1} = W_h(s) h','z_{p2} = W_\theta(s) \theta', 'z_u = W_u(s) \delta_c'};
        SvdAnalysis(CL);
        subplot(2,1,1);
        title('Analiza SVD uk�adu zamkni�tego');
        ylabel('Wektor wyj�ciowy');
        xlabel('\omega [rad/s]');
        subplot(2,1,2);
        xlabel('\omega [rad/s]');
        title('');
        ylabel('Wektor wej�ciowy');
    end
    
    % print -dpsc2 ../doc/Ilustracje/SvdHinf1.ps
    % print -depsc2 ../doc/Ilustracje/SvdHinf1.eps
end

