clear
wing = WingFlutter;
p = 0.8:0.1:1.5;
styles = {'r','g','b'}; s = 1;
%figure; hold on;

%for delta_max_p = [0.3 0.65 1]
for delta_max_p = [1]
    Uf = zeros(size(p));
    for i = 1:length(p)
        wing.U0 = wing.getFlutterSpeed * p(i);
        Gn = wing.getLinearModel;

        K = HinfController3(Gn, 2, 0, delta_max_p);
        ana = LinearAnalysis(K);
    %
    %ana.wing.atmosphere.h = 4000;
    %ana.wing.params.fuelLevel = 1;
        Uf(i) = ana.getFlutterSpeed;
    end

    plot(p,Uf,styles{s});
    s = s+1;
    hold on;
end

legend('0.3','0.65','1');

%%

clear
wing = WingFlutter;
p = 0.8:0.2:1.4;
%p = 1;
styles = {'r--','g--','b--'}; s = 1;
%figure; hold on;

for delta_max_p = 1
    Uf = zeros(size(p));
    for i = 1:length(p)
        Vf = wing.getFlutterSpeed;
        wing.U0 = Vf * p(i);
        %Gn = wing.getLinearModel;

        K = HinfController4(wing, 0, delta_max_p, 2);
        ana = LinearAnalysis(K);
    %
    %ana.wing.atmosphere.h = 4000;
    %ana.wing.params.fuelLevel = 1;
        Uf(i) = ana.getFlutterSpeed(Vf*p(i));
    end

    plot(p,Uf,styles{s});
    s = s+1;
    hold on;
end

%legend('0.3','0.65','1');
