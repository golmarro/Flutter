clear;
wingParams = WingParams();
plane = PlaneParams(wingParams);
wing = WingFlutter(wingParams);
wing.isGravity = 'on';

% U << Uf
Ustall = sqrt(2 * (plane.totalMass/2 * wing.g) ./(wing.atmosphere.rho*wing.params.S*wing.params.alphaMax*wing.params.CLalpha));
Uf = wing.getFlutterSpeed;

fprintf('------------------------Wing U = %f Uf\n', wing.U0 / Uf);
plane.fuelLevel = 0.5;
speed_vec = [];
phase_vec = [];
u = [1.2 * Ustall : 0.05 * Ustall : Uf*1.5];
figure;
hold on;
for f = 0:0.5:1
    wing.params.fuelLevel = f;
    for n = 1:length(u)
        wing.U0 = u(n);
        sys = wing.getModelSS();
        [V E] = eig(sys.A);
        i = 1;   % 3th mode
        mag = abs(V(:,i));
        freq = abs(E(i,i));
        damp = -cos(angle(E(i,i)));
        phase = angle(V(:,i));
        phase = floor(phase*180/pi);

        fprintf('U/Uf = %f, freq = %f, damp = %f, arg(theta-h) = %f\n', wing.U0 / Uf, freq, damp, phase(1) - phase(2));
        speed_vec(n) = wing.U0 / Uf;
        phase_vec(n) = abs(phase(1) - phase(2));
    end
    plot(speed_vec, phase_vec);
end

xlabel('U / U_f');
ylabel('Przesuniêcie fazowe h(t)-\theta(t)');

