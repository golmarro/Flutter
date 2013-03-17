function [Kclassic, descr] = ClassicSISOController(sys)
    % Classi SISO controller for Wing
    if ~exist('sys','var')
        wing = WingFlutter;
        sys = wing.getLinearModel();
    end
    
    %   delta_c  +------+  theta_dot
    % ---------->| wing |------------>
    %            +------+
    
    % See ClassicControlWing
    w_w = 4.19;             % rad/s
    Fw = tf(w_w,[1 w_w]);
    K = 0.2;
    
    Kclassic = -K*Fw;
    Kclassic.InputNames = sys.OutputNames(9);
    Kclassic.OutputNames = sys.InputNames(1);
    descr = 'SISO (theta_dot)';

end

