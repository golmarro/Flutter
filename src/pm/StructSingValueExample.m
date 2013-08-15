%% mussv tests

M = [1 j;1+2j -2];
%M = rand(2) + j*rand(2);
%  | d1  0  |
%  | 0   d2 |
% d1, d2 in C
blk = [-1 0; -1 0];
[bounds, muinfo] = mussv(M,blk);
fprintf('Gorne i dolne ograniczenie wartosci mu:\n')
disp(bounds)
fprintf('dolne i gorne ograniczenie macierzy delta:\n')
disp([1/bounds(1) 1/bounds(2)])

%%
M = rand(4) + j*rand(4);
%  | d1  0  |
%  | 0   d2 |
% d1, d2 in C
blk = [-1 0; -1 0];
[bounds, muinfo] = mussv(M,blk);
fprintf('Gorne i dolne ograniczenie wartosci mu:\n')
disp(bounds)
fprintf('dolne i gorne ograniczenie macierzy delta:\n')
disp([1/bounds(1) 1/bounds(2)])

%% Przyklad z "Control Handbook" str 674 (692)
M1 = [0.1 	-0.154 		0 		0;
0		-0.3		2.86	-26;
0.1		0.077		-0.4	5;
0		-0.004		0.006	0.2;
0.024	0			-0.066	0];
M2 = [0.07		0.162		-0.56	42;
-0.273		-0.28		0.546	72.8;
0.175		-0.108		0.21	3.5;
0.002		-0.002		0.011	0.42;
0.028		0.027		0.042	0.7];
M = M1 + j*M2;
% Przy okazji wplyw struktury delta na wynik:
blk = [1 0; 1 0; 2 3];
[bounds, muinfo] = mussv(M,blk);
bounds
blk = [1 0; 1 0; 1 0; 1 2];
[bounds, muinfo] = mussv(M,blk);
fprintf('Gorne i dolne ograniczenie wartosci mu:\n')
disp(bounds)
fprintf('dolne i gorne ograniczenie macierzy delta:\n')
disp([1/bounds(1) 1/bounds(2)])
% Zgadza sie z tym co w ksiazce

[VDelta,VSigma,VLmi] = mussvextract(muinfo);
fprintf('Wielkosc macierzy zaburzen Delta odpowiada odwrotnosci dolnego ograniczenia _mu = 1/delta_max\n');
disp([norm(VDelta) 1/bounds(2)])
fprintf('Macierz VDelta rzeczywiscie destabilizuje uklad\n');
fprintf('det(I-M Delta) = ');
disp(det(eye(size(M*VDelta))-M*VDelta))