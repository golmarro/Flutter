function Joystick(block)
% Joystick - na wyjsciu wartosc throttle (dlugosc drazka) od 0 do 1 i kat
% przechyenia drazka (w zakresie -0,5  0,5%)


%TODO jak pozbyc sie input zeby nie bylo bledow? Jakies frame czy cos

%%
%% The setup method is used to setup the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.
%%
setup(block);

dlugoscDrazka = 10;
lastPoint = [0 -0.8];
% Oresla czy symulacja biegnie - przelaczane przyciskiem myszy (jesli
% rowne 0 mamy pause)
running = 0;

%endfunction

%% Function: setup ===================================================
%% Abstract:
%%   Set up the S-function block's basic characteristics such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%%
%%   Required         : Yes
%%   C-Mex counterpart: mdlInitializeSizes
%%
function setup(block)

% Register number of ports
block.NumInputPorts  = 0;
block.NumOutputPorts = 1;

% Setup port properties to be inherited or dynamic
%block.SetPreCompInpPortInfoToDynamic;
%block.SetPreCompOutPortInfoToDynamic;
block.SetPreCompPortInfoToDefaults;

% Override input port properties
%block.InputPort(1).Dimensions        = 1;
%block.InputPort(1).DatatypeID  = 0;  % double
%block.InputPort(1).Complexity  = 'Real';
%block.InputPort(1).DirectFeedthrough = true;

% Override output port properties
block.OutputPort(1).Dimensions       = 2;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';

% Register parameters
block.NumDialogPrms     = 0;

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%% -----------------------------------------------------------------
%% The M-file S-function uses an internal registry for all
%% block methods. You should register all relevant methods
%% (optional and required) as illustrated below. You may choose
%% any suitable name for the methods and implement these methods
%% as local functions within the same file. See comments
%% provided for each function for more information.
%% -----------------------------------------------------------------

block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Update', @Update);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Terminate', @Terminate); % Required

end

%%
%% PostPropagationSetup:
%%   Functionality    : Setup work areas and state variables. Can
%%                      also register run-time methods here
%%   Required         : No
%%   C-Mex counterpart: mdlSetWorkWidths
%%
function DoPostPropSetup(block)
  block.NumDworks = 2;
  % W tej zmiennej zapamietujemy handle do axes
  block.Dwork(1).Name            = 'axes';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  % W kolejnej zmiennej Work Vector zapamietamy handle do obiektu line
  % czyli naszego drazka sterujacego
  block.Dwork(2).Name            = 'stick';
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
end

%%
%% Start:
%%   Functionality    : Called once at start of model execution. If you
%%                      have states that should be initialized once, this 
%%                      is the place to do it.
%%   Required         : No
%%   C-MEX counterpart: mdlStart
%%
function Start(block)
    % Przygotowanie obrazu
    figure('CloseRequestFcn',@closeRequest,...
        'WindowButtonDownFcn',@mouseClicked,...
        'Units', 'normalized')
        %'OuterPosition', [0.5 0.4 0.5 0.6]);
    % TODO dostosowac rozmiar figury
    block.Dwork(1).Data = axes('DrawMode','fast');
    axis ([-1 1 -1 1])
    axis equal
    hold
    
    % Przygotowanie "interfejsu" - drazka strujacego i wyswietlacza
    block.Dwork(2).Data = line('XData',[0 lastPoint(1)],'YData',[0 lastPoint(2)],...
        'Marker','p','color','b');
    
    function mouseClicked(src,evnt) %#ok<INUSD>
        % Zatrzymuje i uruchamia symulacje. Jesli uruchomiona - pokazujemy
        % na biezaco aktualne polozenie drazka, jesli nie - drazek znika.
        running = ~running;
        
        if running
            set(src,'pointer','circle')
            set(src,'WindowButtonMotionFcn',@mouseMotion)
        else
            set(src,'pointer','arrow')
            set(src,'WindowButtonMotionFcn','')
            set(block.Dwork(2).Data,'XData',[0 lastPoint(1)],...
                'YData',[0 lastPoint(2)]);
            drawnow
        end

        function mouseMotion(src,evnt) %#ok<INUSD>
            % Tutaj nie robimy nic - chodzi tylko o to, zeby Matlab
            % odswiezyl informacje o polozeniu kursora. Jakiekolwiek
            % operacje w tym miejscu beda wywolywane zbyt czesto!
        end
    end

    function closeRequest(src, evnt) %#ok<INUSD>
        % Do nothing, when simulation terminates the figure will disappear
    end
end


function Update(block)
% Tutaj odswiezamy rysunek (poniewaz update jest wywolywany rzedziej od
% Outputs - mniejsze obciazenie
    if running
        cp = get(block.Dwork(1).Data,'CurrentPoint');
        lastPoint = cp(1,1:2);
    end
    lastPoint(1) = max(lastPoint(1), -1);
    lastPoint(1) = min(lastPoint(1), 1);
    lastPoint(2) = max(lastPoint(2), -1);
    lastPoint(2) = min(lastPoint(2), 1);
    
    set(block.Dwork(2).Data,'XData',[0,lastPoint(1)],'YData',[0,lastPoint(2)]);
    drawnow
end

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C-MEX counterpart: mdlOutputs
%%
function Outputs(block)
    %throttle = norm(lastPoint)/dlugoscDrazka;
    %angle = atan2(lastPoint(2), lastPoint(1))/pi -0.5;

    block.OutputPort(1).Data = [lastPoint(1), lastPoint(2)];
end
%%
%% Update:
%%   Functionality    : Called to update discrete states
%%                      during simulation step
%%   Required         : No
%%   C-MEX counterpart: mdlUpdate
%%
function Terminate(block)
    delete(gcf)
end

end