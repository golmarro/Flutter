

%%
clear
wingParams = WingParams();
plane = PlaneParams(wingParams);
wing = WingFlutter(wingParams);
Uf = wing.getFlutterSpeed();
wing.U0 = 1.01*Uf;
state = wing.trim(0,0);

[t x] = wing.sim(5, [state; 0; 0]);

%%
clear vis
figure
vis = WingVis(wingParams);
vis.init();
vis.scale = 10;
vis.simulate(t,x);