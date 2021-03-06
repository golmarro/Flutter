classdef WingUnc < WingParams
    %WINGUNC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        fl = ureal('FuelLevel',0.5,'Range',[0 1]);
        m_unc
        Itheta_unc
        shtheta_unc
        U0_unc
        
        m_min
        m_max
        I_min
        I_max
        s_min
        s_max
        
        U0_min = 40
        U0_max = 120
    end
    
    methods
        function this = WingUnc()
            params = WingParams();
            params.fuelLevel = 0;
            qmin = [params.m; params.Itheta; params.shtheta];
            params.fuelLevel = 1;
            qmax = [params.m; params.Itheta; params.shtheta];

            
            this.m_min = qmin(1);
            this.I_min = qmin(2); 
            this.s_min = qmin(3);
            
            this.m_max = qmax(1);
            this.I_max = qmax(2); 
            this.s_max = qmax(3);
            
            this.m_unc = qmin(1) + this.fl*(qmax(1) - qmin(1));
            this.Itheta_unc = qmin(2) + this.fl*(qmax(2) - qmin(2));
            this.shtheta_unc = qmin(3) + this.fl*(qmax(3) - qmin(3));
            
            this.U0_unc = ureal('U0',80,'Range',[this.U0_min, this.U0_max]);
        end
        
        function val = getM(this)
            val = this.m_unc;
        end
        
        function val = getItheta(this)
            val = this.Itheta_unc;
        end
        
        function val = getShtheta(this)
            val = this.shtheta_unc;
        end
        
        function val = getOmegah(this)
            val = sqrt(this.Kh / this.m_unc.NominalValue);
        end
        
        function val = getOmegatheta(this)
            val = sqrt(this.Ktheta / this.Itheta_unc.NominalValue);
        end
        
        function val = getU0_unc(this)
            val = this.U0_unc
        end
    end
    
end

