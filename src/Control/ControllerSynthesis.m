classdef ControllerSynthesis
    %CONTROLLERSYNTHESIS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        nomSys          % nominal system
    end
    
    methods
        function this = ControllerSynthesis(nomSys)
            this.nomSys = nomSys;
        end
    end
    
end

