classdef WingVis < handle
    %WINGVIS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Constant)
        airfoil = [1.000000   0.001260
                0.992704   0.002274
                0.979641   0.004079
                0.964244   0.006169
                0.947231   0.008434
                0.929323   0.010765
                0.910956   0.013101
                0.892372   0.015420
                0.873723   0.017700
                0.855041   0.019931
                0.836311   0.022119
                0.817558   0.024266
                0.798819   0.026366
                0.780088   0.028414
                0.761336   0.030413
                0.742560   0.032370
                0.723780   0.034284
                0.705012   0.036149
                0.686255   0.037964
                0.667502   0.039728
                0.648751   0.041440
                0.630004   0.043098
                0.611266   0.044701
                0.592538   0.046245
                0.573821   0.047728
                0.555117   0.049149
                0.536430   0.050503
                0.517763   0.051786
                0.499117   0.052996
                0.480488   0.054127
                0.461875   0.055178
                0.443287   0.056144
                0.424740   0.057019
                0.406241   0.057796
                0.387789   0.058466
                0.369372   0.059023
                0.350989   0.059462
                0.332648   0.059779
                0.314366   0.059965
                0.296159   0.060009
                0.278033   0.059903
                0.259997   0.059634
                0.242060   0.059191
                0.224236   0.058562
                0.206544   0.057733
                0.189011   0.056692
                0.171676   0.055421
                0.154596   0.053909
                0.137852   0.052138
                0.121548   0.050098
                0.105827   0.047785
                0.090903   0.045220
                0.077039   0.042449
                0.064541   0.039548
                0.053594   0.036612
                0.044211   0.033717
                0.036254   0.030913
                0.029567   0.028218
                0.023982   0.025653
                0.019310   0.023217
                0.015371   0.020871
                0.012012   0.018579
                0.009117   0.016316
                0.006653   0.014058
                0.004621   0.011797
                0.003007   0.009544
                0.001777   0.007318
                0.000894   0.005155
                0.000322   0.003059
                0.000036   0.001014
                0.000036  -0.001014];
    end
    
    properties(GetAccess = 'public', SetAccess = 'private')
        wing
        fig
        ax
        transform
        pict
    end
    
    properties(GetAccess = 'public', SetAccess = 'public')
        scale = 6
        timeScale = 0.25
    end
    
    properties(Dependent)
        Xsp
        Xcg
        Xac
    end
    
    methods
        function this = WingVis(wing)
                this.wing = wing;
        end
        
        function val = get.Xcg(this)
            val = this.wing.Xcg_p;
        end
        function val = get.Xsp(this)
            val = this.wing.Xsp_p;
        end
        function val = get.Xac(this)
            val = this.wing.Xac_p;
        end
        
        function init(this)
            this.ax  = axes('XLim',[-1 1],'YLim',[-1 1],...
                'DrawMode','fast');
            grid on; 
            axis equal;
            hold on;

            xdata = this.airfoil(:,1);
            this.pict(1) = line('Xdata',[xdata; xdata],...
                'Ydata',[this.airfoil(:,2); -this.airfoil(:,2)],...
                'color','k');
            this.pict(2) = line('Xdata',[0 1],'Ydata',[0 0],...
                'color','k');
            this.pict(3) = line('Xdata',this.Xsp,'Ydata',0,...
                'LineStyle','none','Marker','x','color','k');
            this.pict(4) = line('Xdata',this.Xsp + this.Xcg,...
                'Ydata',0,...
                'LineStyle','none','Marker','o','color','r');
            this.pict(5) = line('Xdata',this.Xac,...
                'Ydata',0,...
                'LineStyle','none','Marker','.','color','b');
            set(this.pict,'Clipping','off')
            this.transform = hgtransform('Parent',this.ax);
            set(this.pict,'Parent',this.transform)
            set(gcf,'Renderer','opengl')
            
            Sxy = makehgtform('translate', -this.Xsp, 0, 0);
            set(this.transform,'Matrix',Sxy)
            drawnow
        end
        
        function update(this, h, theta)
            h = h*this.scale;
            theta = theta*this.scale;
            Rz = makehgtform('zrotate',-theta);
            Sxy = makehgtform('translate', -this.Xsp, -h, 0);
            set(this.transform,'Matrix',Sxy*Rz)
            drawnow
        end
        
        function simulate(this,t,x)
            fprintf('Simulation\n\tTime scale: %f\n\tMovement scale: %f',...
                this.timeScale, this.scale);
            time = 0;
            timesUpdated = 0;
            i = 1;
            tic
            while time < t(end)
                time = toc * this.timeScale;
                while i<length(t) && t(i) < time
                    i = i+1;
                end
                this.update(x(i,1), x(i,2));
                timesUpdated = timesUpdated + 1;
            end
            fprintf('TimesUpdated: %f, simTime: %f, realTime: %f, fps: %f\n',...
                timesUpdated, t(end), time, timesUpdated/time);
        end
    end
    
end

