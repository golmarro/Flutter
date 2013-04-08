function [Khinf descr] = HinfController1( G , verbose)
    % G - full model
    %% Get full model for flutter speed
    if ~exist('G','var')
        wing = WingFlutter;
    wing.U0 = wing.getFlutterSpeed();
    G = wing.getLinearModel();
    end
    
    
    h_max = 0.1;
    theta_max = 3*pi/180;
    delta_max = 45*pi/180;

    %% Analysis model P
    P = G([1,2,8,9],[1 1]);
    fprintf('Inputs:\n');
    disp(P.InputName)
    fprintf('Outputs:\n')
    disp(P.OutputName)
    P.InputName = ['delta_p';'delta_c'];

    %% Add weights - synthesis model Pw
    Wu = 1/(delta_max*0.3);
    Wp = [1/h_max 0;
          0   1/theta_max];

    systemnames = 'P Wp Wu';
    inputvar = '[delta_p;delta_c]';
    outputvar = '[Wp;Wu;P(3:4)]';
    input_to_P = '[delta_p;delta_c]';
    input_to_Wp = '[P(1:2)]';
    input_to_Wu = '[delta_c]';
    %outputnames = '[zp{2};zu;h_dd;theta_dd]';
    Pw = sysic;

    %Pw = augw(P,Wp, Wu, []);

    %% Synthesis
    [Khinf, CL] = hinfsyn(Pw, 2, 1, 'display', 'on');

    if exist('verbose','var')
        cl = feedback(P, Khinf,1,3:4, 1);

        systemnames = '[P Khinf]';
        inputvar = '[delta_p]';
        outputvar = '[P(1:2);Khinf]';
        input_to_P = '[delta_p;Khinf]';
        input_to_Khinf = '[P(3:4)]';
        cl2 = sysic;
    end

    Khinf.InputName = P.OutputName(3:4);
    Khinf.OutputName = G.inputName(1);
    
    fprintf('Khinf inputs:\n');
    disp(Khinf.InputName)
    fprintf('Khinf outputs:\n')
    disp(Khinf.OutputName)
    
    Khinf = -Khinf;
    descr = 'Hinf 1';
end

