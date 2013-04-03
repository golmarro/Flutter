function [  ] = SvdAnalysis(sys, settings)
%SVDANALYSIS Analyse system's worst case response using SVD decomposition

    if ~exist('settings','var')
        settings = '';
    end
    % Only to get some initial range for omega:
    [sv w] = sigma(sys);
    
    % Get a series of transfer function matrices sys(jw)
    fr = freqresp(sys, w);       
    % Now fr(:,:,i) == sys(jw(i))
    
    % Init wectors for results
    sigma_max = zeros(1,length(w));
    output = zeros(size(sys,1),length(w));
    input = zeros(size(sys,2),length(w));

    for i = 1:size(fr,3);
        % Do the SVD decomposition
        [u s v] = svd(fr(:,:,i));
        % now:
        %    sys(jw) = u*s*v'
        %    sys(jw)*v = u*s
        % max output = u(:,1) * s(1,1)
        
        % We are interested in worst case, so take first vectors and first
        % sigma
        sigma_max(i) = s(1,1);
        output(:,i) = u(:,1) * sigma_max(i);
        input(:,i) = v(:,1);
    end

    % Plot results
    if nargout == 0
        styles = {'r:','g:','b:','r--','g--','b--'};
        figure;
        subplot(2,1,1);
        semilogx(w,sigma_max,'k');
        hold on;
        leg = cell(size(sys,1)+1,1);
        leg{1} = '\sigma_{max} (C_l(j\omega))';
        for i = 1:size(sys,1)
            semilogx(w,abs(output(i,:)),styles{i});
            leg(i+1) = sys.OutputName(i);
        end
        legend(leg)
        title('Worst case output');

        subplot(2,1,2);
        % it's separately only because we cannot do hold on before,
        % otherwise the plot will be broken (bug)
        semilogx(w,abs(input(1,:)),styles{1});
        hold on;
        leg = cell(size(sys,2),1);
        leg(1) = sys.InputName(1);
        for i = 2:size(sys,2)
            semilogx(w,abs(input(i,:)),styles{i});
            leg(i) = sys.InputName(i);
        end
        title('Worst case input');
        legend(leg);
        
        if ~isempty(findstr(settings,'phase'))
            figure
            
            subplot(2,1,1);
            semilogx(w,angle(output(1,:))*180/pi,styles{1});
            hold on;
            leg = cell(size(sys,1),1);
            leg(1) = sys.OutputName(1);
            for i = 2:size(sys,1)
                semilogx(w,angle(output(i,:))*180/pi,styles{i});
                leg(i) = sys.OutputName(i);
            end
            legend(leg)
            title('Worst case output - phase');

            subplot(2,1,2);
            % it's separately only because we cannot do hold on before,
            % otherwise the plot will be broken (bug)
            semilogx(w,angle(input(1,:))*180/pi,styles{1});
            hold on;
            leg = cell(size(sys,2),1);
            leg(1) = sys.InputName(1);
            for i = 2:size(sys,2)
                semilogx(w,angle(input(i,:))*180/pi,styles{i});
                leg(i) = sys.InputName(i);
            end
            title('Worst case input - phase');
            legend(leg);
        end
    end
end

