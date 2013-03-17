function [K, descr] = ClassicSISOController(sys)
    % Classi SISO controller for Wing
    if ~exist('sys','var')
        wing = WingFlutter;
        sys = wing.getLinearModel();
    end
    
    %   delta_c  +------+  theta_dot
    % ---------->| wing |------------>
    %            +------+
    K = [0 -1];
    descr = 'SISO (theta_dot)';

end

