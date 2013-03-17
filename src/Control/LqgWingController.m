function [Klqg descr] = LqgWingController( sys )
    % LQG regulator for Wing Flutter
    if ~exist('sys','var')
        wing = WingFlutter;
        sys = wing.getLinearModel();
        wing.U0 = wing.getFlutterSpeed();
    end
    
    % Extract model for synthesis from full model:
    
    %    delta_c    +----+ -----> h_dot
    %  ------------>|wing|
    %               +----+ -----> theta_dot
    sys = sys(3:4,1);
    
    % Weighting matrices
    % state and control signal costs
    %           h  theta             delta_c
    %           |   |                 |
    Qxu = diag([10 10 0 0 0 0 0 0   0.01]);  
    %             8 states (x)    + 1 input (u)
    
    % process noise (state disturbance) and measurement noise covariance
    %              h_dot  theta_dot  turbulence
    %                 |     |         |    h_dot theta_dot
    Qwv = diag([0 0 6.377 3.268 0 0 0 1    0.1     0.1]);          
    %               8 states (w)         + 2 measurements - outputs (v)
    Klqg = lqg(sys, Qxu, Qwv);
    
    % Positive feeback is required (see lqg function reference)
    Klqg = -Klqg;
    
    descr = 'LQG';
end

