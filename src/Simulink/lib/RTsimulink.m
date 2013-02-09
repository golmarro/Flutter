function RTsimulink(block)
% S-function dla bloku Real Time. Spowalnia symulacje do czasu
% rzeczywistego.

  setup(block);
  
end

function setup(block)
  
  %% Register number of dialog parameters   
  block.NumDialogPrms = 0;

  %% Register number of input and output ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 1;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions       = 1;
  
  %% Set block sample time to continuous
  block.SampleTimes = [0 0];

  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
  block.RegBlockMethod('Start',                   @Start);
  block.RegBlockMethod('InitializeConditions',    @InitConditions);  
  block.RegBlockMethod('Outputs',                 @Output);
  block.RegBlockMethod('Terminate',               @Terminate);
  
end

function DoPostPropSetup(block)
%% Tutaj deklarujemy zmienne uzywane przez S-funkcje. Zmienna pass
% okresla, czy mozna juz przejsc do kolejnego punktu kontrolnego.
% Punkt kontrolny - moment w ktorym wstrzymujemy symulacje, az czas
% rzeczywisty zrowna sie z czasem symulacyjnym.
% Druga zmienna to timer - zapamietany - poniewaz pozniej musimy go
% zastopowac (funkcja Terminate)
  block.NumDworks = 1;
  block.Dwork(1).Name            = 'pass';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
end

function InitConditions(block)
  block.Dwork(1).Data = 0;
  
end

function Start(block)
  tic
end

function Output(block)
  while block.InputPort(1).Data > toc
      if block.InputPort(1).Data - toc > 0.001
        pause(block.InputPort(1).Data - toc)
      end
  end
  block.OutputPort(1).Data = toc;
end

function Terminate(block)
end