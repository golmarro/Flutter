

%%
clear
wingParams = WingParams();
plane = PlaneParams(wingParams);
wing = WingFlutter(wingParams);
Uf = wing.getFlutterSpeed();
wing.U0 = 0.95*Uf;
state = wing.trim(0,0);

[t x] = wing.sim(5, [state; 0; 0]);

%%
clear vis
vis = WingVis(wingParams);
vis.init();
vis.scale = 1;
vis.simulate(t,x);