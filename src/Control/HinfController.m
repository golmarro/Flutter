function [Khinf descr] = HinfController(sys)
    %Hinf controller based on bact robust control by Waszak
    if ~exist('sys','var')
        wing = WingFlutter;
        sys = wing.getLinearModel();
        wing.U0 = wing.getFlutterSpeed();
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
    
    h_max = 0.1;
    theta_max = 3*pi/180;
    delta_max = 45*pi/180;
    
    if 0
        % w = [eta (turb), delta_p]
        % u = [delta_c]
        % z = [h, theta, delta]
        % y = [h_dot, theta_dot]
        sys = sys([1 2 5 3 4],[2 1 1]);
        Wout = diag([0.1 10*pi/180 45*pi/180 1 1]);
        sys = Wout*sys;
        [Khinf, CL] = hinfsyn(sys, 2, 1, 'display', 'on');
    else
        % w = [eta (turb)]
        % u = [delta_c]
        % z = [h, theta, delta]
        % y = [h_dot, theta_dot]
        P = sys([1 2 5 3 4],[2 1]);
        Wout = diag([1/h_max 1/theta_max 1/delta_max 1 1]);
        Pw = Wout*P;
        [Khinf, CL] = hinfsyn(Pw, 2, 1, 'display', 'on');
        Khinf = -Khinf;
    end
    
    
    
    descr = 'Hinf';
end

