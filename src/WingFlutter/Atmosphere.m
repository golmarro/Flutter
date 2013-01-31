classdef Atmosphere < handle
    %ATMOSPHERE Simplified standard atmosphere model (troposphere only)
    %   Detailed explanation goes here
    
    properties(Constant)
        g  = 9.80665;       % Earth-surface gravitational acceleration [m/s2]
        T0 = 288.15;        % Sea level standard temperature [K]
        p0 = 101325;        % Sea level standard atmospheric pressure [Pa]
        M  = 0.0289644;     % Molar mass of dry air [kg/mol]
        R  = 8.31447;       % Universal gas constant [J/(mol*K)]
        L  = -0.0065;       % Temperature lapse rate [K/m]
    end
    properties(Dependent)
        c                   % Speed of sound [m/s]
        p                   % Pressure [N/m2]
        rho                 % Density [kg/m3]
        T                   % Temp [K]
        TC                  % Temp [C]
    end
    properties(GetAccess = 'public', SetAccess = 'public')
        h = 4000;           % Altitude [m]
    end
    
    methods
        function this = Atmosphere(h)
            if exist('h','var')
                this.h = h;
            end
        end
        
        function set.h(this, h)
            this.h = h;
            if h<0 || h>11000
                warning('Altitude out of range: %f [m]', h);
            end
        end
        
        function c = get.c(this)
            c = 331.3 * sqrt(1 + this.TC/273.15);
        end
        
        function set.c(this, t_c)
            this.TC = ((t_c / 331.3)^2 - 1)* 273.15;
        end
        
        function p = get.p(this)
            p = this.p0*exp( - this.g * this.M * this.h / ( this.R * this.T0 ) );
        end
        
        function set.p(this, t_p)
            this.h = - (this.R * this.T0) / ( this.g * this.M ) * log(t_p/this.p0);
        end
        
        function rho = get.rho(this)
            rho = this.p * this.M/(this.R * this.T);
        end
        
%         function set.rho(this, t_rho)
%             
%         end
        
        function T = get.T(this)
            T = this.T0 + this.L*this.h;
        end
        
        function set.T(this,t_T)
            this.h = (t_T - this.T0) / this.L;
        end
        
        function TC = get.TC(this)
            TC = this.T - 273.15;
        end
        
        function set.TC(this, t_TC)
            this.T = t_TC + 273.15;
        end
        
        function val = getProp(this, name)
            val = this.(name);
        end
        
        function disp(this)
            fprintf('Atmosphere:\n');
            fprintf('\tAltitude: %f [m]\n', this.h);
            fprintf('\tPressure: %f [Pa]\n', this.p);
            fprintf('\tDensity: %f [kg/m3]\n', this.rho);
            fprintf('\tTemp: %f [C]\n', this.TC);
            fprintf('\tSpeed of sound: %f [m/s]\n', this.c);
        end
    end
    
    methods(Static)
        function plot()
            atm = Atmosphere;
            h_range = 0:500:11000;
            for i = 1:length(h_range)
                atm.h = h_range(i);
                params(i,:) = [atm.TC, atm.p, atm.rho, atm.c];
            end
            figure; hold; 
            title('Atmosphere')
            ylabel('altitude [m]')
            plot(params(:,1), h_range, 'r');
            plot(params(:,2)/1000, h_range, 'b');
            plot(params(:,3), h_range, 'g');
            plot(params(:,4), h_range, 'y');
            legend('T [C]', 'p [kPa]', '\rho [kg/m^3]', 'c [m/s]');
            grid
        end
    end
    
end

