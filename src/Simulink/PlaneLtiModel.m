classdef PlaneLtiModel < handle
    %PLANELTIMODEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        speed;
        fuel;
        alt;
        sysStiff;
        opStiff;
        sysFlex;
        opFlex;
    end
    
    methods
        function this = PlaneLtiModel()   % Constructor
            try
                load PlaneLtiModel
                this = planeLtiModel;
            catch me
            end
        end
        
        function save(this)
            planeLtiModel = this;
            save PlaneLtiModel planeLtiModel;
        end
        
        function linearize(this,simRunner, speed, fuel, alt)
            if ~exist('simRunner','var')
                error('Please create simRunner object in workspace');
            end
            if ~exist('fuel','var')
                %fuel = [0 0.5 1];
                fuel = 0.5;
            end
            if ~exist('speed','var')
                wing = WingFlutter;
                Vk = wing.getFlutterSpeed();
                speed = Vk*(0.6:0.05:1.7);
            end
            if ~exist('alt','var')
                %alt = [0 5500 11000];
                alt = 5500;
            end
            
            this.speed = speed;
            this.fuel = fuel;
            this.alt = alt;
            
%             close_system('PlaneStiffSim',0)
%             close_system('PlaneSim',0)

            [this.sysStiff this.opStiff] = this.linearizePrivate(speed,fuel,alt,'PlaneStiffSim',simRunner);
            [this.sysFlex this.opFlex] = this.linearizePrivate(speed,fuel,alt,'PlaneSim',simRunner);            
            
            this.save;
        end
        
        function comparePzmaps(this, fi, hi)
            figure; hold on;
            %fi = 1;
            %hi = 1;
            for vi = 1:length(this.speed)
                [~, lambda] = eig(this.sysStiff{vi, fi, hi}.a);
                lambda = diag(lambda);
                if vi == 1
                    style = 'ro';
                elseif vi == length(this.speed)
                    style = 'bo';
                else
                    style = 'ko';
                end
                handle = plot(real(lambda),imag(lambda), style);
                hasbehavior(handle,'legend',false);
            end
            hasbehavior(handle,'legend',true);
            for vi = 1:length(this.speed)
                [~, lambda] = eig(this.sysFlex{vi, fi, hi}.a);
                lambda = diag(lambda);
                if vi == 1
                    style = 'rx';
                elseif vi == length(this.speed)
                    style = 'bx';
                else
                    style = 'kx';
                end
                handle = plot(real(lambda),imag(lambda), style);
                hasbehavior(handle,'legend',false);
            end
            hasbehavior(handle,'legend',true);
            
            ylim = get(gca,'YLim');
            xlim = get(gca,'XLim');
            handle = plot(xlim,[0 0], 'k:');
            hasbehavior(handle,'legend',false);
            handle = plot([0 0],ylim, 'k:');
            hasbehavior(handle,'legend',false);
            
            legend('Model sztywny','Model polsztywny');
        end
        
        function [sysArray opArray] =  linearizePrivate(this, speed, fuel, alt, mdl, simRunner)
            
            sysArray = cell(length(speed), length(fuel),length(alt));
            opArray =  cell(length(speed), length(fuel),length(alt));
            
            for fi = 1:length(fuel);
                %simRunner.wingFlutter.params.fuelLevel = fuel(fi);
                for hi = 1:length(alt)
                    %simRunner.wingFlutter.atmosphere.h = alt(hi);
                    for ui = 1:length(speed)
                        close_system('PlaneStiffSim',0)
                        close_system('PlaneSim',0)
                        
                        %simRunner = PlaneRunner('simRunner', mdl);
                        simRunner.mdl = mdl;
                        % simRunner.actuatorModel = 'none';
                        
                        simRunner.wingFlutter.params.fuelLevel = fuel(fi);
                        simRunner.wingFlutter.atmosphere.h = alt(hi);
                        simRunner.wingFlutter.U0 = speed(ui);
                        simRunner.trim();
                        opArray{ui,fi,hi} = simRunner.lastOperPoint;
                        sys = simRunner.linearize();
                        if strcmp(mdl,'PlaneSim')
                            sys = sys(1:17,:);
                            sys.StateName = {...
                           'Lw_theta','Lw_h','Rw_theta','Rw_h'...
                           ,'theta','psi','-phi','x1','x2','x3'...
                           ,'Lw_d_theta','Lw_d_h','Rw_d_theta','Rw_d_h'...
                           ,'d_theta','d_psi','-d_phi','v1','v2','v3'...
                           ,'actRS','actRS','actLS','actLS'...
                           ,'LW_turb1','LW_turb2'...
                           ,'actLA','actLA'...
                           ,'RW_turb3','RW_turb4'...
                           ,'actRA','actRA'};
                        else
                            %sys = sys(1:17,:);
                        end
                        % size(sys.StateName)   
                        sysArray{ui,fi,hi} = sys;
                    end
                end
            end
        end
    end
    
end

